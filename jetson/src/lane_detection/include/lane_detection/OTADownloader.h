#pragma once

#include <string>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class Downloader {
public:
    Downloader(const std::string& downloadFolder);
    void downloadFile(const std::string& serverIp, const std::string& port, const std::string& filePath);

private:
    std::string downloadFolder_;
};

void DownloadFirmware(void);
void SendFirmware(void);