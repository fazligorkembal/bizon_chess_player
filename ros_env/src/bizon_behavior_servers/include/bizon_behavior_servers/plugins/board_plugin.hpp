#ifndef BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP
#define BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"

#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_behavior_servers/debug_session.hpp"
#include "bizon_msgs/action/board.hpp"

#include <opencv2/opencv.hpp>

#include "bizon_behavior_servers/plugins/cell_detector.hpp"
#include "bizon_behavior_servers/plugins/cell_classifier.hpp"
using namespace nvinfer1;

namespace bizon_behaviors
{
    using BoardAction = bizon_msgs::action::Board;

    class BoardPlugin : public TimedBehavior<BoardAction>
    {
    public:
        using BoardActionGoal = BoardAction::Goal;

        /**
         * @brief A constructor for behavior_server::Board
         */
        BoardPlugin();
        ~BoardPlugin();

        /**
         * @brief Reads debug_session_dir and registers this plugin's image
         * dump callback with DebugSession -- see the header comment on
         * writeImageArtifacts() for why BoardPlugin is the one that has to
         * register it.
         */
        void onConfigure() override;

        /**
         * @brief Initialization to run behavior
         * @param command Goal to execute
         * @return Status of behavior
         */
        ResultStatus onRun(const std::shared_ptr<const BoardActionGoal> command) override;

        /**
         * @brief Loop function to run behavior
         * @return Status of behavior
         */
        ResultStatus onCycleUpdate() override;

        /**
         * @brief Called on action completion to fill result
         * @param result Result to fill
         */
        void onActionCompletion(std::shared_ptr<BoardAction::Result> result) override;

    protected:
        rclcpp::Time board_end_;
        BoardAction::Feedback::SharedPtr feedback_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        cv::Mat latest_image_;
        mutable std::mutex image_mutex_;
        void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        std::atomic<bool> is_camera_active_{false};
        CellDetector cell_detector_{1280, 720}; // TODO: Make these parameters configurable
        // Geometry must match the weights file. yolo26n is the nano variant:
        // gd 0.50, gw 0.25, max_channels 1024 (see parse_args in tensorrt/utils.h).
        // The medium values that used to sit here asked for a 64-channel first conv
        // against nano weights holding 16, so engine building aborted.
        CellClassifier cell_classifier_{"/home/user/Documents/bizon_chess_player/models/yolo26n-chessboard_v2.wts", 0.50f, 0.25f, 1024, "n"};
        std::string results_;
        int32_t empty_count_{0};
        std::atomic<bool> is_result_ready_{false};

        // FEN debounce. Two robots share one board, and the camera happily
        // photographs it in the middle of the other arm's move -- a capture is
        // two physical moves, so there is a window where the captured piece is
        // already gone and the capturing piece has not arrived. That position
        // is not reachable by any legal move, the decision ladder rightly finds
        // no match, and the goal fails. Requiring the same FEN on several
        // consecutive frames means the board has stopped changing before we act
        // on it, which is also the interlock that keeps an arm out of a square
        // the other arm is still working in.
        std::string pending_fen_;
        int pending_fen_count_{0};
        int fen_stable_count_{3};

        // A debounce that can wait forever is a hang, not a filter. The
        // classifier flickers by a cell now and then, so "three identical in a
        // row" is not guaranteed to ever happen. After this many inferences in
        // one goal we accept the latest FEN and say so, which degrades to the
        // old single-frame behaviour instead of leaving the arm parked at home.
        int fen_settle_attempts_{30};
        int fen_attempts_this_goal_{0};
        bool is_black_side_{false};

        // ---- Error-bundle state (spec section 5). Everything below is
        // guarded by image_mutex_, same as latest_image_ -- cameraCallback
        // writes it, and both BoardPlugin's own board-detect-failure path
        // and DebugSession's image-dump callback (invoked from
        // DecisionPlugin, on a different thread/process-plugin) read it. ----

        // Last kFrameRingSize whole camera frames, oldest first, dropping
        // the oldest on push. "frame_0" in a bundle is frame_ring_.back().
        static constexpr size_t kFrameRingSize = 3;
        std::deque<cv::Mat> frame_ring_;
        void pushFrame(const cv::Mat & frame);

        // Snapshot of the most recent successful board read, used to fill
        // an error bundle triggered either here (board_detect_failed, where
        // this data is NOT available yet -- see writeImageArtifacts) or
        // from DecisionPlugin via the registered callback (fen_mismatch,
        // no_move_owner, where it reflects the board read that produced the
        // mismatched camera FEN).
        cv::Mat last_image_cropped_;
        std::vector<cv::Rect> last_bboxes_;
        // Cell crops and per-cell predictions in bbox/scan order (i.e.
        // *before* the is_black_side_ reversal cameraCallback applies to
        // build the FEN) -- see debug_format::squareForCellIndex() for how
        // a bundle recovers each crop's actual board square from this order.
        std::vector<cv::Mat> last_cell_images_;
        std::vector<std::string> last_class_names_pre_reversal_;
        std::vector<float> last_confidences_;
        bool last_is_black_side_{false};
        bool has_cell_data_{false};

        // Registered with DebugSession in onConfigure(); also called
        // directly (skipping the frames-only special case) for a
        // board-detect failure. Writes, in the priority order the spec
        // calls out (context.txt is written by the caller before this
        // runs; images matter less than the timeline): cells_overlay.png,
        // then cells_contact_sheet.png, then frames/. Each step is
        // independently best-effort -- one image failing to write must
        // never stop the rest, and must never fail the calling goal.
        void writeImageArtifacts(const std::string & bundle_dir);
        // Board detection itself failed, so there is no cropped image or
        // cell data to bundle -- only the frame ring buffer and a short
        // context.txt BoardPlugin writes itself.
        void dumpBoardDetectFailureBundle();
    };
}

#endif // BIZON_BEHAVIOR_SERVERS_PLUGINS_BOARD_PLUGIN_HPP