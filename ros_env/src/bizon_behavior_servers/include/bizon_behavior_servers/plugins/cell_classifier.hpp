#ifndef CALL_CLASSIFIER_HPP
#define CALL_CLASSIFIER_HPP

#include <algorithm>
#include <cmath>

#include "bizon_behavior_servers/tensorrt/cuda_utils.h"
#include "bizon_behavior_servers/tensorrt/types.h"
#include "bizon_behavior_servers/tensorrt/utils.h"
#include "bizon_behavior_servers/tensorrt/logging.h"
#include "bizon_behavior_servers/tensorrt/model.h"

Logger gLogger;
using namespace nvinfer1;

namespace bizon_behaviors
{

    class CellClassifier
    {
    public:
        CellClassifier(const std::string wts_path, float gd, float gw, int max_channels, std::string type, int32_t output_size = 13)
            : gd_(gd), gw_(gw), max_channels_(max_channels), type_(type), kOutputSize(output_size)
        {

            std::string engine_name = wts_path;
            size_t lastindex = engine_name.find_last_of(".");
            engine_name = engine_name.substr(0, lastindex) + ".engine";

            if (!std::ifstream(engine_name))
            {
                RCLCPP_INFO(rclcpp::get_logger("CellClassifier"), "Serialized engine file not found at %s. Building engine...", engine_name.c_str());
                serialize_engine(wts_path, engine_name, gd_, gw_, max_channels_, type_);
            }

            RCLCPP_INFO(rclcpp::get_logger("CellClassifier"), "Serialized engine file found at %s. Deserializing engine...", engine_name.c_str());
            deserialize_engine(engine_name, &runtime_, &engine_, &context_);
            CUDA_CHECK(cudaStreamCreate(&stream_));
            prepare_buffer(engine_, &input_buffer_device_, &output_buffer_device_, &input_buffer_host_, &output_buffer_host_);
            device_buffers_[0] = input_buffer_device_;
            device_buffers_[1] = output_buffer_device_;
        }

        // confidences is optional (debug-bundle cell_labels.txt is the only
        // caller that wants it -- see the "confidence if available" line in
        // the spec); passing nullptr skips the softmax pass entirely so the
        // hot classification path pays nothing for it.
        void infer(std::vector<cv::Mat> &imgs, std::vector<std::string> &results, std::vector<float> *confidences = nullptr)
        {
            batch_preprocess(imgs, input_buffer_host_, kClsInputW, kClsInputH);
            CUDA_CHECK(cudaMemcpyAsync(input_buffer_device_, input_buffer_host_, kBatchSize * 3 * kClsInputH * kClsInputW * sizeof(float), cudaMemcpyHostToDevice, stream_));
            context_->enqueueV2((void **)device_buffers_, stream_, nullptr);
            CUDA_CHECK(cudaMemcpyAsync(output_buffer_host_, output_buffer_device_, kBatchSize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream_));
            cudaStreamSynchronize(stream_);

            for (size_t i = 0; i < kBatchSize; i++)
            {
                std::vector<float> logits(output_buffer_host_ + i * kOutputSize, output_buffer_host_ + (i + 1) * kOutputSize);
                int top_index = topk(logits, 1)[0];
                std::string class_name = class_names_[top_index];
                results.push_back(class_name);

                if (confidences != nullptr)
                {
                    confidences->push_back(softmaxConfidence(logits, top_index));
                }
            }
        }

    private:
        void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine,
                                IExecutionContext **context)
        {
            std::ifstream file(engine_name, std::ios::binary);
            if (!file.good())
            {
                RCLCPP_ERROR(rclcpp::get_logger("BoardPlugin"),
                             "Failed to read TensorRT engine file: '%s'. "
                             "Please verify the file exists and has read permissions.",
                             engine_name.c_str());
                throw std::runtime_error("Failed to read TensorRT engine file");
            }
            size_t size = 0;
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            char *serialized_engine = new char[size];
            assert(serialized_engine);
            file.read(serialized_engine, size);
            file.close();

            *runtime = createInferRuntime(gLogger);
            assert(*runtime);
            *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
            assert(*engine);
            *context = (*engine)->createExecutionContext();
            assert(*context);
            delete[] serialized_engine;
            RCLCPP_INFO(rclcpp::get_logger("BoardPlugin"),
                        "Successfully deserialized TensorRT engine from file: '%s'.",
                        engine_name.c_str());
        }

        void serialize_engine(const std::string &wts_name, std::string &engine_name, float &gd, float &gw, int &max_channels,
                              std::string &type)
        {
            IBuilder *builder = createInferBuilder(gLogger);
            IBuilderConfig *config = builder->createBuilderConfig();
            IHostMemory *serialized_engine =
                buildEngineYolo26Cls(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels, type);

            assert(serialized_engine);
            std::ofstream p(engine_name, std::ios::binary);
            if (!p)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BoardPlugin"),
                             "Failed to open output file for TensorRT engine serialization: '%s'.",
                             engine_name.c_str());
                assert(false);
            }
            p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());
            RCLCPP_INFO(rclcpp::get_logger("BoardPlugin"),
                        "Successfully serialized TensorRT engine to file: '%s'.",
                        engine_name.c_str());
            delete serialized_engine;
            delete config;
            delete builder;
        }

        void batch_preprocess(std::vector<cv::Mat> &imgs, float *output, int dst_width = 224, int dst_height = 224)
        {
            for (size_t b = 0; b < imgs.size(); b++)
            {
                int h = imgs[b].rows;
                int w = imgs[b].cols;
                int m = std::min(h, w);
                int top = (h - m) / 2;
                int left = (w - m) / 2;
                cv::Mat img = imgs[b](cv::Rect(left, top, m, m));
                cv::resize(img, img, cv::Size(dst_width, dst_height), 0, 0, cv::INTER_LINEAR);
                cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
                img.convertTo(img, CV_32F, 1 / 255.0);

                std::vector<cv::Mat> channels(3);
                cv::split(img, channels);

                // CHW format
                for (int c = 0; c < 3; ++c)
                {
                    int i = 0;
                    for (int row = 0; row < dst_height; ++row)
                    {
                        for (int col = 0; col < dst_width; ++col)
                        {
                            output[b * 3 * dst_height * dst_width + c * dst_height * dst_width + i] =
                                channels[c].at<float>(row, col);
                            ++i;
                        }
                    }
                }
            }
        }

        void prepare_buffer(ICudaEngine *engine, float **input_buffer_device, float **output_buffer_device,
                            float **input_buffer_host, float **output_buffer_host)
        {
            assert(engine->getNbBindings() == 2);
            // In order to bind the buffers, we need to know the names of the input and output tensors.
            // Note that indices are guaranteed to be less than IEngine::getNbBindings()
            const int inputIndex = engine->getBindingIndex(kInputTensorName);
            const int outputIndex = engine->getBindingIndex(kOutputTensorName);
            assert(inputIndex == 0);
            assert(outputIndex == 1);

            // Create GPU buffers on device
            CUDA_CHECK(cudaMalloc((void **)input_buffer_device, kBatchSize * 3 * kClsInputH * kClsInputW * sizeof(float)));
            CUDA_CHECK(cudaMalloc((void **)output_buffer_device, kBatchSize * 13 * sizeof(float)));

            *input_buffer_host = new float[kBatchSize * 3 * kClsInputH * kClsInputW];
            *output_buffer_host = new float[kBatchSize * 13];
            RCLCPP_INFO(rclcpp::get_logger("BoardPlugin"),
                        "Successfully prepared GPU buffers for TensorRT engine. Input buffer size: %zu bytes, Output buffer size: %zu bytes.",
                        kBatchSize * 3 * kClsInputH * kClsInputW * sizeof(float),
                        kBatchSize * 13 * sizeof(float));
        }

        // Softmax probability of `top_index` among `logits`. Only computed
        // when the caller asks for confidences (see infer()'s confidences
        // parameter) -- it is extra work over raw logits the hot path
        // doesn't otherwise need.
        float softmaxConfidence(const std::vector<float> &logits, int top_index)
        {
            const float max_logit = *std::max_element(logits.begin(), logits.end());
            float sum_exp = 0.0f;
            for (float logit : logits)
            {
                sum_exp += std::exp(logit - max_logit);
            }
            if (sum_exp <= 0.0f)
            {
                return 0.0f;
            }
            return std::exp(logits[top_index] - max_logit) / sum_exp;
        }

        std::vector<int> topk(const std::vector<float> &vec, int k)
        {
            std::vector<int> topk_index;
            std::vector<size_t> vec_index(vec.size());
            std::iota(vec_index.begin(), vec_index.end(), 0);

            std::sort(vec_index.begin(), vec_index.end(),
                      [&vec](size_t index_1, size_t index_2)
                      { return vec[index_1] > vec[index_2]; });

            int k_num = std::min<int>(vec.size(), k);

            for (int i = 0; i < k_num; ++i)
            {
                topk_index.push_back(vec_index[i]);
            }

            return topk_index;
        }

        float gd_;
        float gw_;
        int max_channels_;
        std::string type_;
        cv::Mat image_gray_;
        IRuntime *runtime_ = nullptr;
        ICudaEngine *engine_ = nullptr;
        IExecutionContext *context_ = nullptr;
        cudaStream_t stream_;

        // Prepare cpu and gpu buffers
        float *device_buffers_[2];
        float *input_buffer_device_ = nullptr;
        float *output_buffer_device_ = nullptr;
        float *input_buffer_host_ = nullptr;
        float *output_buffer_host_ = nullptr;
        int kOutputSize = 13;
        std::vector<std::string> class_names_ = {"b", "k", "n", "p", "q", "r", "emp", "B", "K", "N", "P", "Q", "R"}; // TODO: MAKE IT CONFIGURABLE
    };
}

#endif // CALL_CLASSIFIER_HPP