#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <iostream>
#include <map>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <csignal>

const cv::Point2d REAL_SIZE(28.0, 15.0);

cv::Point GroundSize;
cv::Mat OriginalGround, DrawedGround;
std::map<size_t, std::pair<cv::Point2d, std::string>> Enemies;
std::string draw_topic = "radar_debug";
boost::iostreams::filtering_ostream out;
size_t recieve_count = 0;

void draw_point(cv::Point2d pt, int tag);

class MqttClient : public mosqpp::mosquittopp {
public:
    explicit MqttClient(const char *id) : mosqpp::mosquittopp(id) {}

    void on_message(const struct mosquitto_message* message) override
    {
        recieve_count++;
        if (recieve_count % 1000 == 0) {
            out.flush();
            std::cout << "Recieve " << recieve_count << " messages" << std::endl;
        }
        std::cout << "*" << message->topic << " " << boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time()) << " " << (char*)message->payload << std::endl;
        out << "*" << message->topic << " " << boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time()) << " " << (char*)message->payload << std::endl;
        if (message->topic != draw_topic)
            return;
        std::string payload(static_cast<char *>(message->payload), message->payloadlen);
        auto json = nlohmann::json::parse(payload);
        auto pos = json["pos"].get<std::vector<double>>();
        auto tag = json["id"].get<size_t>();
        auto msg = json["msg"].get<std::string>();
        Enemies[tag] = {{pos[0], pos[1]}, msg};
    }
};

void draw_point(std::pair<cv::Point2d, std::string> pt, int tag) {
    std::cout << "Draw point: " << pt.first << " " << pt.second << " " << tag << std::endl;
    cv::Scalar color;
    size_t r_tag;
    if (tag < 100)  // 红
        color = cv::Scalar(0, 0, 255), r_tag = tag;
    else            // 蓝
        color = cv::Scalar(255, 0, 0), r_tag = tag - 100;
    cv::Point center(GroundSize.x * pt.first.x / REAL_SIZE.x, GroundSize.y * (1 - pt.first.y / REAL_SIZE.y));
    cv::circle(DrawedGround, center, 10, color / 2, -1);
    cv::circle(DrawedGround, center, 10, color, 1);
    cv::putText(DrawedGround, std::to_string(r_tag), center + cv::Point(-6, 4), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                {255, 255, 255}, 1);
    cv::putText(DrawedGround, pt.second, center + cv::Point(-6, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
}

int main(int argc, char** argv)
{
    auto json = nlohmann::json::parse(std::ifstream("config.json"));
    draw_topic = json["draw_topic"].get<std::string>();

    std::string time_str = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
    std::string filename_str = time_str + ".mqttdump.zst";

    out.push(boost::iostreams::zstd_compressor());
    out.push(boost::iostreams::file_sink(filename_str, std::ios_base::binary | std::ios_base::out));
    std::cout << "Dumping to " << filename_str << std::endl;

    std::signal(SIGINT, [](int) {
        std::cout << "Exiting..." << std::endl;
        out.flush();
        out.reset();
        std::exit(0);
    });  // Handle Ctrl-C

    mosqpp::lib_init();
    MqttClient client("draw");

    client.connect(json["address"].get<std::string>().c_str());

    for (auto &topic: json["topics"]) {
        client.subscribe(nullptr, topic.get<std::string>().c_str(), 0);
    }

    OriginalGround = cv::imread("ground.png");
    GroundSize = OriginalGround.size();
    DrawedGround = OriginalGround.clone();
    cv::namedWindow("Result", cv::WINDOW_AUTOSIZE);
    while (true) {
        DrawedGround = OriginalGround.clone();
        for (auto &[tag, pos]: Enemies)
            draw_point(pos, tag);
        cv::imshow("Result", DrawedGround);
        int key = cv::pollKey();
        if (key > 0) {
            Enemies.clear();
        }
        // std::cout << "Loop" << std::endl;
        client.loop(1, 14);
    }
    mosqpp::lib_cleanup();
    return 0;
}
