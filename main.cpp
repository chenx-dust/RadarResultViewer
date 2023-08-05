#include <map>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

const cv::Point2d REAL_SIZE(28.0, 15.0);

cv::Point GroundSize;
cv::Mat OriginalGround, DrawedGround;
std::map<size_t, std::pair<cv::Point2d, std::string>> Enemies;

void draw_point(cv::Point2d pt, int tag);

class MqttClient : public mosqpp::mosquittopp {
public:
    explicit MqttClient(const char *id) : mosqpp::mosquittopp(id) {}

    void on_message(const struct mosquitto_message *message) override {
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

int main(int argc, char **argv) {
    mosqpp::lib_init();
    MqttClient client("draw");
    if (argc == 2)
        client.connect(argv[1]);
    else
        client.connect("192.168.1.5");
    client.subscribe(nullptr, "radar_debug", 0);
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
