#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include <string>
#include <curl/curl.h>

class HttpClient {
public:
    HttpClient();
    ~HttpClient();

    // Sends a GET request
    std::string get(const std::string& url);

    // Sends a POST request with JSON payload
    std::string post(const std::string& url, const std::string& jsonData);

private:
    // Helper to capture response data
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);
};

#endif