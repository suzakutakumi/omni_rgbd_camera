#include <cnoid/BodyItem>
#include <cnoid/Camera>
#include <cnoid/Item>
#include <cnoid/ItemList>
#include <cnoid/Joystick>
#include <cnoid/RangeCamera>
#include <cnoid/RootItem>
#include <cnoid/SimpleController>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <omp.h>

#include "PointCloud.hpp"

using namespace cnoid;
using namespace std;

class OmniRGBDCameraController : public SimpleController
{
    RangeCamera *omniRangeCamera[4];
    std::ostream *os;
    SimpleControllerIO *io;
    Joystick joystick;
    bool PrevButtonState;

    Link *root;

    SimpleControllerConfig *config;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_publisher;

    double init_pos[3];

public:
    virtual bool configure(SimpleControllerConfig *config) override
    {
        cout << "start configure" << endl;
        node = rclcpp::Node::make_shared("omni_rgbd_camera_node");
        sensor_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 10);

        this->config = config;
        return true;
    }

    virtual bool initialize(SimpleControllerIO *io) override
    {
        cout << "start initalize" << endl;

        this->io = io;
        os = &io->os();

        omniRangeCamera[0] = io->body()->findDevice<RangeCamera>(
            "RangeCameraFront");
        omniRangeCamera[1] = io->body()->findDevice<RangeCamera>(
            "RangeCameraRight");
        omniRangeCamera[2] = io->body()->findDevice<RangeCamera>(
            "RangeCameraLeft");
        omniRangeCamera[3] = io->body()->findDevice<RangeCamera>(
            "RangeCameraBack");

        for (auto camera : omniRangeCamera)
        {
            io->enableInput(camera);
        }

        root = io->body()->rootLink();
        root->setActuationMode(Link::LinkPosition);
        io->enableInput(root);
        io->enableOutput(root);

        return true;
    }

    cnoid::Isometry3 pos;
    virtual bool start() override
    {
        cout << "start" << endl;
        pos = root->position();
        init_pos[0] = pos.translation().x();
        init_pos[1] = pos.translation().y();
        init_pos[2] = pos.translation().z();
        cout << "start2" << endl;
        return true;
    }
    virtual bool control() override
    {
        // cout << "control" << endl;
        joystick.readCurrentState();

        static bool PrevMoveButtonState[4] = {false, false, false, false};
        const cnoid::Joystick::ButtonID ButtonState[4] = {cnoid::Joystick::Y_BUTTON,
                                                          cnoid::Joystick::B_BUTTON,
                                                          cnoid::Joystick::X_BUTTON,
                                                          cnoid::Joystick::A_BUTTON};

        for (int i = 0; i < 4; i++)
        {
            bool Button = joystick.getButtonState(ButtonState[i]);
            if (Button && !PrevMoveButtonState[i])
            {
                switch (i)
                {
                case 0:
                    pos.translation().x() += 0.1;
                    break;
                case 1:
                    pos.translation().y() -= 0.1;
                    break;
                case 2:
                    pos.translation().y() += 0.1;
                    break;
                case 3:
                    pos.translation().x() -= 0.1;
                    break;
                default:
                    break;
                }

                root->setTranslation(pos.translation());
                // cout<<"New Position: "<<pos.translation().transpose()<<endl;

                // item->getLocationProxy()->setGlobalLocation(pos);
            }
            PrevMoveButtonState[i] = Button;
        }

        bool buttonState = joystick.getPosition(2) != 0.0;
        static int timer = 0;
        timer++;
        if (timer >= 60 * 60)
        {
            timer = 0;
            static auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
            const auto &timestamp = clock.now();

            cout << "merge pointcloud" << endl;
            // auto merged = getPointCloud(pos.translation().x() - init_pos[0], pos.translation().y() - init_pos[1], pos.translation().z() - init_pos[2]);
            auto merged = getPointCloud();

            pcl::PointCloud<pcl::PointXYZRGB> merged_pc = *merged.get_cloud();
            merged_pc.is_dense = false;
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(merged_pc.makeShared());
            sor.setLeafSize(0.05, 0.05, 0.05);
            sor.filter(merged_pc);

            cout << "publish pointcloud" << endl;
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(merged_pc, pcl_msg);
            pcl_msg.header.set__stamp(timestamp);
            pcl_msg.header.set__frame_id("nemui");
            pcl_msg.is_dense = false;
            sensor_publisher->publish(pcl_msg);
        }
        PrevButtonState = buttonState;
        return true;
    }

    PointCloud getPointCloud(double mx = 0, double my = 0, double mz = 0)
    {
        std::string name_directions[4] = {"front", "right", "left", "back"};
        pcl::PointCloud<pcl::PointXYZRGB> clouds[4];

        //  現在のシーンの画像取得
#pragma omp parallel for
        for (auto direction = 0; direction < 4; direction++)
        {
            auto camera = omniRangeCamera[direction];

            const Image &RangeImage = camera->constImage();
            //  現在のシーンの画像保存
            // RangeImage.save("pointcloud" + name_directions[direction]);

            //  画像の高さと横幅
            const int width = RangeImage.width();
            const int height = RangeImage.height();
            //  色データを取得
            const unsigned char *pixels = RangeImage.pixels();

            // Point Cloudの変数宣言
            pcl::PointCloud<pcl::PointXYZRGB> &cloud = clouds[direction];
            // Point Cloudの初期化
            cloud.width = width;
            cloud.height = height;
            cloud.is_dense = false;
            cloud.points.resize(cloud.width * cloud.height);

            std::size_t i = 0;

            //  Point Cloudに各点の値（座標、色）を格納
            #pragma omp parallel for
            for (const auto &e : camera->constPoints())
            {
                // X,Y,Zを格納
                cloud[i].x = e(0);
                cloud[i].y = e(1);
                cloud[i].z = e(2);
                //  色（R,G,B）を格納
                cloud[i].r = pixels[3 * i + 0];
                cloud[i].g = pixels[3 * i + 1];
                cloud[i].b = pixels[3 * i + 2];
                ++i;
            }
        }

        PointCloud pcs[4];
#pragma omp parallel for
        for (auto direction = 0; direction < 4; direction++)
        {
            pcs[direction] = PointCloud(clouds[direction]);
        }

        enum Direction
        {
            Front,
            Right,
            Left,
            Back,
        };

#pragma omp parallel for
        for (int i = 0; i < 4; i++)
        {
            // pcs[i].z_range_filter(-3.0, 3.0);

            switch (i)
            {
            case Direction::Front:
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              { p.z -= 0.080; });
                break;
            case Direction::Back:
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                    p.x = -p.x;
                    p.z = -p.z;
                    p.z += 0.080; });
                break;
            case Direction::Right:
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                    float x = p.x;
                    p.x = -p.z;
                    p.z = x;
                    p.x += 0.080; });
                break;
            case Direction::Left:
                pcs[i].filter([](pcl::PointXYZRGB &p)
                              {
                    float x = p.x;
                    p.x = p.z;
                    p.z = -x;
                    p.x -= 0.080; });
                break;
            }
        }

        PointCloud merged;
        for (auto &p : pcs)
        {
            merged.extended(p);
        }

        merged.filter([=](pcl::PointXYZRGB &p)
                      {
            p.x += mx;
            p.y += mz;
            p.z += -my; });

        return merged;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OmniRGBDCameraController)