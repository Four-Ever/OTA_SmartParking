#include "lane_detection/OTADownloader.h"
#include <fstream>
#include <iostream>
#include <boost/version.hpp>
#include "lane_detection/CentralGateway.h"

Downloader::Downloader(const std::string& downloadFolder)
    : downloadFolder_(downloadFolder) {}

void Downloader::downloadFile(const std::string& serverIp, const std::string& port, const std::string& filePath) {
    try {
        net::io_context io_context;
        tcp::resolver resolver(io_context);
        tcp::socket socket(io_context);

        auto const results = resolver.resolve(serverIp, port);
        net::connect(socket, results.begin(), results.end());

        http::request<http::string_body> req{http::verb::get, "/upload/" + filePath, 11};
        req.set(http::field::host, serverIp);
        req.set(http::field::user_agent, "Boost.Beast/" BOOST_LIB_VERSION); 

        http::write(socket, req);

        beast::flat_buffer buffer;
        http::response<http::dynamic_body> res;
        http::read(socket, buffer, res);

        if (res.result() != http::status::ok) {
            std::cerr << "다운로드 실패: " << res.result() << "\n";
            return;
        }

        std::string body = beast::buffers_to_string(res.body().data());
        std::string fullFilePath = downloadFolder_ + "/" + filePath;

        std::ofstream outputFile(fullFilePath, std::ios::binary);
        if (!outputFile) {
            std::cerr << "다운로드 파일 생성 실패: " << fullFilePath << "\n";
            return;
        }

        outputFile << body;
        outputFile.close();

        std::cout << "File '" << filePath << "' 다운로드 성공 '" << fullFilePath << "'.\n";

        beast::error_code ec;
        socket.shutdown(tcp::socket::shutdown_both, ec);
    } catch (std::exception const& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }
}

void DownloadFirmware(void) {
    std::cout<<"서버로부터 다운로드드"<<std::endl;
    std::string serverIp = "192.168.203.197"; 
    std::string port = "5000";          
    //std::string canInterface = "can0";  
    std::string downloadFolder = "/home/jetson/sj/auto-drivingpjt/jetson/Firmware";  // 다운로드 위치 변경됨

    std::string fileName = "PART_A.bin";

    Downloader downloader(downloadFolder);
    downloader.downloadFile(serverIp, port, fileName);

    //std::string fullFilePath = downloadFolder + "/" + fileName;

    // can
    //Sender sender(canInterface);
    //sender.sendFile(fullFilePath);
}

bool SendFirmware(void) {
    std::string file_path = "/home/jetson/sj/auto-drivingpjt/jetson/Firmware/PART_A.bin";
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return false;
    }

    // Get file size
    file.seekg(0, std::ios::end);
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Calculate padded size (multiple of 8)
    std::streamsize padded_size = std::ceil(file_size / 8.0) * 8;
    
    std::cout << "Original file size: " << file_size << " bytes" << std::endl;
    std::cout << "Padded file size: " << padded_size << " bytes" << std::endl;

    // Send control command (ID: 0x59)
    // struct can_frame frame;
    const uint8_t control_command[8] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    
    // frame.can_id = 0x59;
    // frame.can_dlc = 8;
    // std::memcpy(frame.data, control_command, 8);

    //CGW_OTA_Control_ID
    auto ota_control_msg = std::make_shared<CGW_OTA_Control_Msg>();
    std::memcpy(&(ota_control_msg->data_.data.ota_control), control_command, 8);
    
    // if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
    //     std::cerr << "Failed to send control command" << std::endl;
    //     return false;
    // }

    if (CGW->can_socket_->send(ota_control_msg) == false) {
        std::cerr << "Failed to send control command" << std::endl;
        return false;
    }

    std::cout << "Control command sent" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));


    auto file_size_msg = std::make_shared<CGW_OTA_File_Size_Msg>();
    file_size_msg->SetOtaFileSize(padded_size);

    // if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
    //     std::cerr << "Failed to send file size" << std::endl;
    //     return false;
    // }

    if (CGW->can_socket_->send(file_size_msg) == false) {
        std::cerr << "Failed to send file size" << std::endl;
        return false;
    }

    std::cout << "File size sent"<< padded_size << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Send file data (ID: 0x5D)
    const int chunk_size = 8;
    char buffer[chunk_size];
    std::streamsize total_sent = 0;
    auto ota_file_data_msg = std::make_shared<CGW_OTA_File_Data_Msg>();

    int previous_progress_ = 0;
    while (file.read(buffer, chunk_size) || file.gcount() > 0) {
        //frame.can_dlc = 8;  // Always send 8 bytes, pad with zeros if needed
        std::memset(&(ota_file_data_msg->data_.data.ota_file_data), 0, 8);  // Clear previous data
        //std::memcpy(frame.data, buffer, file.gcount());
        std::memcpy(&(ota_file_data_msg->data_.data.ota_file_data), buffer, file.gcount());

        // if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        //     std::cerr << "Failed to send data chunk" << std::endl;
        //     return false;
        // }

        if (CGW->can_socket_->send(ota_file_data_msg) == false) {
            std::cerr << "Failed to send file size" << std::endl;
            return false;
        }

        total_sent += ota_file_data_msg->GetSizeCan();
        
        // if (total_sent % 1024 == 0) {  // Show progress every 1KB
        //     float progress = (float)total_sent / padded_size * 100;
        //     std::cout << "\rProgress: " << progress << "%" << std::flush;
        // }

        if (total_sent % 1024 == 0) {
            int current_progress = (int)((total_sent * 100) / padded_size);
            if (current_progress != previous_progress_) {  // 진행률이 변경되었을 때만 출력
                std::cout << "\rProgress: " << current_progress << "%" << std::flush;
                previous_progress_ = current_progress;

                std::shared_ptr<IMessage> wmsg = std::make_shared<CGW_OTA_Update_State_Msg>();
                auto request = std::static_pointer_cast<CGW_OTA_Update_State_Msg>(wmsg);
                request->SetOtaUpdateProgress(current_progress);

                auto ws_session = CGW->ws_server_->current_session_;
                if (ws_session) {
                    // 세션의 strand를 통해 메시지 전송
                    // auto ws_strand = ws_session->get_strand();  // strand getter 필요
                    // boost::asio::post(ws_strand, 
                    //     [ws_session, wmsg]() {
                    //         ws_session->send_message(wmsg);
                    //     });
                    ws_session->send(wmsg);
                }

            }
        }



        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1ms delay between frames
    }

    std::shared_ptr<IMessage> wmsg = std::make_shared<CGW_OTA_Update_State_Msg>();
    auto request = std::static_pointer_cast<CGW_OTA_Update_State_Msg>(wmsg);
    request->SetOtaUpdateProgress(100);

    auto ws_session = CGW->ws_server_->current_session_;
    if (ws_session) {
        // 세션의 strand를 통해 메시지 전송
        // auto ws_strand = ws_session->get_strand();  // strand getter 필요
        // boost::asio::post(ws_strand, 
        //     [ws_session, wmsg]() {
        //         ws_session->send_message(wmsg);
        //     });
        ws_session->send(wmsg);
    }

    std::cout << "\nTotal sent: " << total_sent << " bytes" << std::endl;
    std::cout << "File transmission completed" << std::endl;
    return true;
}