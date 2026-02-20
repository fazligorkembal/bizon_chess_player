#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/wait.h>
#include <cstring>
#include <vector>
#include <sstream>

std::vector<std::string> get_all_legal_moves(
    const std::string &fen,
    int to_sf_fd,
    int from_sf_fd)
{
    std::vector<std::string> moves;

    auto send_command = [&](const std::string &cmd)
    {
        write(to_sf_fd, cmd.c_str(), cmd.size());
    };

    auto read_until = [&](const std::string &stop)
    {
        std::string output;
        char c;
        while (read(from_sf_fd, &c, 1) == 1)
        {
            output += c;
            if (output.find(stop) != std::string::npos)
                break;
        }
        return output;
    };

    // Pozisyonu ayarla
    std::string pos_cmd = "position fen " + fen + "\n";
    send_command(pos_cmd);

    // Legal move listesi al
    send_command("go perft 1\n");

    std::string output = read_until("Nodes searched:");

    std::stringstream ss(output);
    std::string line;

    while (std::getline(ss, line))
    {
        auto colon_pos = line.find(':');
        if (colon_pos != std::string::npos)
        {
            std::string move = line.substr(0, colon_pos);
            if (!move.empty())
                moves.push_back(move);
        }
    }

    return moves;
}

bool is_valid_transition(const std::string &last_fen,
                         const std::string &current_fen,
                         int to_sf_fd,
                         int from_sf_fd)
{
    auto legal_moves = get_all_legal_moves(last_fen, to_sf_fd, from_sf_fd);

    auto send_command = [&](const std::string &cmd)
    {
        write(to_sf_fd, cmd.c_str(), cmd.size());
    };

    auto read_until = [&](const std::string &stop)
    {
        std::string output;
        char c;
        while (read(from_sf_fd, &c, 1) == 1)
        {
            output += c;
            if (output.find(stop) != std::string::npos)
                break;
        }
        return output;
    };

    for (const auto &move : legal_moves)
    {
        std::string cmd = "position fen " + last_fen + " moves " + move + "\n";
        send_command(cmd);

        send_command("d\n");
        std::string board_output = read_until("Checkers:");

        auto fen_pos = board_output.find("Fen: ");
        if (fen_pos == std::string::npos)
            continue;

        std::string test_fen = board_output.substr(fen_pos + 5);
        test_fen = test_fen.substr(0, test_fen.find('\n'));

        if (test_fen == current_fen)
        {
            std::cout << "Legal move found: " << move << std::endl;
            return true;
        }
    }

    return false;
}

std::string get_move_type(
    const std::string &fen,
    const std::string &move)
{
    // --- 1) Board kısmını al ---
    std::string boardPart = fen.substr(0, fen.find(' '));

    // --- 2) FEN alanlarını parse et ---
    std::stringstream ss(fen);
    std::string token;
    std::string activeColor, castleRights, enPassant;
    ss >> token;          // board
    ss >> activeColor;    // w veya b
    ss >> castleRights; // KQkq veya -
    ss >> enPassant;      // en passant karesi veya -

    // --- 3) Rok kontrolu ---
    // activeColor FEN buffer'dan dolayi guvenilir olmayabilir,
    // kaynak karedeki tasi kontrol ederek rok tespiti yap
    std::string from = move.substr(0, 2);
    std::string to = move.substr(2, 2);

    // Kaynak karedeki tasi bul
    int fromFile = from[0] - 'a';
    int fromRank = 8 - (from[1] - '0');
    int fromIndex = fromRank * 8 + fromFile;
    int idx = 0;
    char sourcePiece = '.';
    for (char c : boardPart)
    {
        if (c == '/')
            continue;
        if (isdigit(c))
        {
            if (idx + (c - '0') > fromIndex)
                break;
            idx += (c - '0');
        }
        else
        {
            if (idx == fromIndex)
            {
                sourcePiece = c;
                break;
            }
            idx++;
        }
    }

    // Beyaz sah (K) e1'den rok yapiyor
    if (sourcePiece == 'K' && from == "e1")
    {
        if (to == "g1" && castleRights.find('K') != std::string::npos)
            return "short_castle";
        if (to == "c1" && castleRights.find('Q') != std::string::npos)
            return "long_castle";
    }
    // Siyah sah (k) e8'den rok yapiyor
    if (sourcePiece == 'k' && from == "e8")
    {
        if (to == "g8" && castleRights.find('k') != std::string::npos)
            return "short_castle";
        if (to == "c8" && castleRights.find('q') != std::string::npos)
            return "long_castle";
    }

    // --- 4) Promotion kontrolu ---
    if (move.size() == 5)
    {
        int file = move[2] - 'a';
        int rank = 8 - (move[3] - '0');
        int targetIndex = rank * 8 + file;
        int currentIndex = 0;
        bool targetOccupied = false;

        for (char c : boardPart)
        {
            if (c == '/')
                continue;
            if (isdigit(c))
            {
                if (currentIndex + (c - '0') > targetIndex)
                    break;
                currentIndex += (c - '0');
            }
            else
            {
                if (currentIndex == targetIndex)
                {
                    targetOccupied = true;
                    break;
                }
                currentIndex++;
            }
        }

        if (targetOccupied)
            return "promotion_capture";
        return "promotion";
    }

    // --- 5) Hedef kareyi hesapla ---
    int file = move[2] - 'a';
    int rank = 8 - (move[3] - '0');
    int targetIndex = rank * 8 + file;

    // --- 6) FEN icinde hedef kareyi bul ---
    int currentIndex = 0;

    for (char c : boardPart)
    {
        if (c == '/')
            continue;

        if (isdigit(c))
        {
            int empty = c - '0';
            if (currentIndex + empty > targetIndex)
                break;
            currentIndex += empty;
        }
        else
        {
            if (currentIndex == targetIndex)
                return "capture";
            currentIndex++;
        }
    }

    // --- 7) En passant kontrol ---
    std::string targetSquare = move.substr(2, 2);
    if (targetSquare == enPassant)
        return "en_passant";

    return "straight";
}

int main()
{
    // Stockfish kurulu mu kontrol et
    int ret = system("which stockfish > /dev/null 2>&1");
    if (ret != 0)
    {
        std::cerr << "Error: stockfish not found! Please install it: sudo apt install stockfish" << std::endl;
        return 1;
    }
    std::cout << "stockfish found, starting..." << std::endl;

    int to_sf[2], from_sf[2];
    pipe(to_sf);
    pipe(from_sf);

    const std::string short_rok_white = "e1g1";
    const std::string long_rok_white = "e1c1";
    const std::string short_rok_black = "e8g8";
    const std::string long_rok_black = "e8c8";

    pid_t pid = fork();

    if (pid == 0)
    {
        close(to_sf[1]);
        close(from_sf[0]);
        dup2(to_sf[0], STDIN_FILENO);
        dup2(from_sf[1], STDOUT_FILENO);
        close(to_sf[0]);
        close(from_sf[1]);
        execlp("stockfish", "stockfish", nullptr);
        perror("execlp failed");
        return 1;
    }

    // Parent process
    close(to_sf[0]);
    close(from_sf[1]);

    auto send_command = [&](const char *cmd)
    {
        write(to_sf[1], cmd, strlen(cmd));
    };

    auto read_until = [&](const std::string &stop) -> std::string
    {
        std::string output;
        char c;
        while (read(from_sf[0], &c, 1) == 1)
        {
            output += c;
            if (output.size() >= stop.size() &&
                output.substr(output.size() - stop.size()) == stop)
            {
                break;
            }
        }
        return output;
    };

    send_command("d\n");
    send_command("isready\n");
    std::string board = read_until("readyok\n");
    std::cout << "Stockfish output:\n"
              << board << std::endl;

    // Baslangic pozisyonundan kendi kendine oynat
    std::string moves = "";
    int turn = 0;
    moves = "";
    turn = 0;
    std::string last_fen = "";
    std::string current_fen = "";
    while (true)
    {
        std::string pos_cmd = "position startpos";
        if (!moves.empty())
            pos_cmd += " moves" + moves;
        pos_cmd += "\n";
        send_command(pos_cmd.c_str());

        if (turn % 2 == 1)
        {
            // send_command("setoption name UCI_LimitStrength value false\n"); // Beyaz full power
            // send_command("setoption name Skill Level value 20\n");
            // send_command("go depth 15\n");
            send_command("go depth 10\n");
        }
        else
        {
            send_command("setoption name Skill Level value 0\n");
            send_command("setoption name UCI_LimitStrength value true\n");
            send_command("setoption name UCI_Elo value 800\n");
            send_command("go depth 5\n");
        }

        std::string output = read_until("bestmove");

        std::string bestmove_line;
        char c;
        while (read(from_sf[0], &c, 1) == 1)
        {
            bestmove_line += c;
            if (c == '\n')
                break;
        }

        std::string move = bestmove_line.substr(1);
        move.erase(move.find_last_not_of(" \n\r") + 1);
        if (move.find(' ') != std::string::npos)
            move = move.substr(0, move.find(' '));

        if (move == "(none)")
        {
            std::cout << "Game over. Winner: " << (turn % 2 == 1 ? "White" : "Black") << std::endl;
            break;
        }

        std::string pos_cmd2 = "position startpos";
        if (!moves.empty())
            pos_cmd2 += " moves" + moves;
        pos_cmd2 += "\n";
        send_command(pos_cmd2.c_str());

        // FEN almak için d komutu gönder
        send_command("d\n");

        // Gerçek FEN'i oku
        std::string board_output = read_until("Checkers:");

        auto fen_pos = board_output.find("Fen: ");
        if (fen_pos != std::string::npos)
        {
            last_fen = board_output.substr(fen_pos + 5);
            last_fen = last_fen.substr(0, last_fen.find('\n'));
        }
        else
        {
            std::cout << "FEN bulunamadi!\n";
            continue;
        }

        turn++;
        moves += " " + move;
        std::string move_type = get_move_type(last_fen, move);
        std::cout << "Current FEN: " << last_fen << " --- Hamle " << turn << " (" << (turn % 2 == 1 ? "Beyaz" : "Siyah") << "): " << move << " (" << move_type << ")" << std::endl;

        send_command("d\n");
        send_command("isready\n");

        if (last_fen != "" && last_fen != current_fen)
        {
            current_fen = last_fen;
        }
    }

    send_command("quit\n");
    close(to_sf[1]);
    close(from_sf[0]);
    waitpid(pid, nullptr, 0);

    return 0;
}