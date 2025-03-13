#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <cnoid/Joystick>
#include <cnoid/RangeCamera>
#include <cnoid/SimpleController>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iostream>
// #include <omp.h>

// #include "PointCloud.hpp"

using namespace cnoid;
using namespace std;

class OmniRGBDCameraController : public SimpleController
{
    RangeCamera *omniRangeCamera[4];
    std::ostream *os;
    SimpleControllerIO *io;
    Joystick joystick;
    bool PrevButtonState;

    // Link *root = nullptr;

    SimpleControllerConfig *config;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_publisher;

    // double init_pos[3];

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

        // root = io->body()->rootLink();
        // if (!root)
        // {
        //     cout << "failed" << endl;
        //     return false;
        // }
        // root->setActuationMode(Link::LinkPosition);
        // io->enableInput(root);
        // io->enableOutput(root);

        // cout << "ggggaaa" << endl;

        // pos = root->position().translation().eval().cast<double>();
        // cout << "hgfghgf" << endl;

        // cout << root->name() << endl;
        // cout << pos.x() << endl;
        // // pos = cnoid::Translation3(Eigen::Vector3d(root->translation()));
        // init_pos[1] = 1.0;
        // init_pos[2] = 1.0;
        // cout << "end" << endl;

        return true;
    }

    // cnoid::Affine3 rotation;
    // cnoid::Vector3d pos;
    // std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    virtual bool start() override
    {
        cout << "start" << endl;
        return true;
    }
    virtual bool control() override
    {
        // cout << "control" << endl;
        joystick.readCurrentState();

        // static bool PrevMoveButtonState[4] = {false, false, false, false};
        // const cnoid::Joystick::ButtonID ButtonState[4] = {cnoid::Joystick::Y_BUTTON,
        //                                                   cnoid::Joystick::B_BUTTON,
        //                                                   cnoid::Joystick::X_BUTTON,
        //                                                   cnoid::Joystick::A_BUTTON};

        // for (int i = 0; i < 4; i++)
        // {
        //     bool Button = joystick.getButtonState(ButtonState[i]);
        //     if (Button && !PrevMoveButtonState[i])
        //     {
        //         switch (i)
        //         {
        //         case 0:
        //             pos.x() += 0.1;
        //             break;
        //         case 1:
        //             pos.y() -= 0.1;
        //             break;
        //         case 2:
        //             pos.y() += 0.1;
        //             break;
        //         case 3:
        //             pos.x() -= 0.1;
        //             break;
        //         default:
        //             break;
        //         }

        //         root->setTranslation(pos);
        //         // cout<<"New Position: "<<pos.translation().transpose()<<endl;

        //         // item->getLocationProxy()->setGlobalLocation(pos);
        //     }
        //     PrevMoveButtonState[i] = Button;
        // }

        bool buttonState = joystick.getPosition(2) != 0.0;
        static auto timer = clock.now();
        if (stop_flg and (clock.now() - timer).seconds() > 2.0)
        {
            cout << "aaa" << endl;
            // timer = 0;
            if (th.joinable())
            {
                th.join();
            }
            else
            {
                stop_flg = false;
                timer = clock.now();
                cout << "b" << endl;
                th = std::thread(std::bind(&OmniRGBDCameraController::publishPointCloud, this));
                cout << "c" << endl;
            }
        }
        PrevButtonState = buttonState;
        return true;
    }

    virtual void stop() override
    {
        th.join();
    }

    std::thread th = std::thread([]() {});
    bool stop_flg = true;
    rclcpp::Clock clock = rclcpp::Clock(RCL_SYSTEM_TIME);
    void publishPointCloud()
    {
        const auto &timestamp = clock.now();

        cout << "merge pointcloud" << endl;
        // auto merged = getPointCloud(pos.translation().x() - init_pos[0], pos.translation().y() - init_pos[1], pos.translation().z() - init_pos[2]);
        auto merged_pc = getPointCloud();
        pcl::PointCloud<pcl::PointXYZRGB> filterd_pc;

        cout << "voxel filter" << endl;

        merged_pc->is_dense = false;
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(merged_pc);
        sor.setLeafSize(0.05, 0.05, 0.05);
        sor.filter(filterd_pc);

        cout << "publish pointcloud" << endl;
        sensor_msgs::msg::PointCloud2::UniquePtr pcl_msg(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(filterd_pc, *pcl_msg);
        pcl_msg->header.set__stamp(timestamp);
        pcl_msg->header.set__frame_id("nemui");
        pcl_msg->is_dense = merged_pc->is_dense;
        sensor_publisher->publish(std::move(pcl_msg));

        stop_flg = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(double mx = 0, double my = 0, double mz = 0)
    {
        std::string name_directions[4] = {"front", "right", "left", "back"};
        pcl::PointCloud<pcl::PointXYZRGB> clouds[4];

        enum Direction
        {
            Front,
            Right,
            Left,
            Back,
        };

        //  現在のシーンの画像取得
        // #pragma omp parallel for
        for (auto direction = 0; direction < 4; direction++)
        {
            auto camera = omniRangeCamera[direction];

            const Image &RangeImage = camera->constImage();

            //  画像の高さと横幅
            const int width = RangeImage.width();
            const int height = RangeImage.height();
            //  色データを取得
            const unsigned char *pixels = RangeImage.pixels();

            // // Point Cloudの変数宣言
            pcl::PointCloud<pcl::PointXYZRGB> &cloud = clouds[direction];
            // Point Cloudの初期化
            cloud.width = width;
            cloud.height = height;
            cloud.is_dense = false;
            cloud.points.resize(cloud.width * cloud.height);

            std::size_t i = 0;

            //  Point Cloudに各点の値（座標、色）を格納
            // #pragma omp parallel for
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

            double theta = 0;
            switch (direction)
            {
            case Direction::Front:
                theta = 0;
                break;
            case Direction::Back:
                theta = M_PI;
                break;
            case Direction::Right:
                theta = 3 * M_PI / 2;
                break;
            case Direction::Left:
                theta = M_PI / 2;
                break;
            }
            Eigen::Affine3f move_transform = Eigen::Affine3f::Identity();
            move_transform.translation() << 0.0, 0.0, -0.080;

            Eigen::Affine3f rotate_transform = Eigen::Affine3f::Identity();
            rotate_transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

            pcl::transformPointCloud(cloud, cloud, rotate_transform * move_transform);
        }

        pcl::PointCloud<pcl::PointXYZRGB> merged;
        for (auto &cloud : clouds)
        {
            merged += cloud;
        }

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << mx, my, mz;

        pcl::transformPointCloud(merged, merged, transform);

        return merged.makeShared();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OmniRGBDCameraController)