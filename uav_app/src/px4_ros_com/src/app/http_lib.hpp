#include <env_parser.hpp>
#include <curl/curl.h>
#include <string>

class HTTPExecutor {
    public:
        HTTPExecutor();
        void http_post(std::string data_path, std::string env_path, std::string type, std::string url_path);

    private:
        CURL* curl;
        EnvParser env_parser;
        std::string type = "Content-Type: image/jpg";
};

HTTPExecutor::HTTPExecutor() {}

size_t read_callback(void *ptr, size_t size, size_t nmemb, void *data) {
    std::ifstream *file = static_cast<std::ifstream *>(data);
    file->read(reinterpret_cast<char *>(ptr), size * nmemb); 
    return file->gcount(); // Return the number of bytes read
}

void HTTPExecutor::http_post(
    std::string data_path, 
    std::string env_path, 
    std::string type, 
    std::string url_path
) {

    curl = curl_easy_init();
    if (curl) {

        std::ifstream file(data_path);
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }

        env_parser.load_env(env_path);
        env_parser.parse_env();

        std::string supabase_auth = env_parser.get_var("SUPABASE_AUTH");
        std::string supabase_url = env_parser.get_var("SUPABASE_URL");

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, supabase_auth.c_str());
        headers = curl_slist_append(headers, type.c_str());

        std::string supabase_data_url = supabase_url + url_path;
        curl_easy_setopt(curl, CURLOPT_URL, supabase_data_url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
        curl_easy_setopt(curl, CURLOPT_READDATA, &file);
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK) 
            std::cerr << "POST REQUEST FAILED" << std::endl;

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        file.close();
    }

}