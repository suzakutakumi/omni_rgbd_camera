// #define EIGEN_MAX_ALIGN_BYTES 0
#include <cnoid/Joystick>
#include <cnoid/SimpleController>

#include <iostream>

using namespace cnoid;
using std::endl;

class MoveController : public SimpleController
{
    std::ostream *os;
    SimpleControllerIO *io;
    Joystick joystick;
    bool PrevButtonState;

    Link *root = nullptr;

    SimpleControllerConfig *config;

    double init_pos[3];

public:
    virtual bool configure(SimpleControllerConfig *config) override
    {
        this->config = config;
        os = &config->os();
        *os << "start configure" << endl;

        return true;
    }

    virtual bool initialize(SimpleControllerIO *io) override
    {
        *os << "start initalize" << endl;

        this->io = io;
        os = &io->os();

        root = io->body()->rootLink();
        if (!root)
        {
            *os << "failed" << endl;
            return false;
        }
        root->setActuationMode(Link::LinkPosition);
        io->enableInput(root);
        io->enableOutput(root);

        *os << "ggggaaa" << endl;

        pos = root->position().translation().eval().cast<double>();

        *os << root->name() << endl;
        *os << pos.x() << endl;
        // pos = cnoid::Translation3(Eigen::Vector3d(root->translation()));
        init_pos[1] = 1.0;
        init_pos[2] = 1.0;
        *os << "end" << endl;

        return true;
    }

    cnoid::Affine3 rotation;
    cnoid::Vector3d pos;
    // std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    virtual bool start() override
    {
        *os << "start" << endl;
        return true;
    }
    virtual bool control() override
    {
        // *os << "control" << endl;
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
                    pos.x() += 0.1;
                    break;
                case 1:
                    pos.y() -= 0.1;
                    break;
                case 2:
                    pos.y() += 0.1;
                    break;
                case 3:
                    pos.x() -= 0.1;
                    break;
                default:
                    break;
                }

                root->setTranslation(pos);
                // *os<<"New Position: "<<pos.translation().transpose()<<endl;

                // item->getLocationProxy()->setGlobalLocation(pos);
            }
            PrevMoveButtonState[i] = Button;
        }

        return true;
    }

    virtual void stop() override
    {
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MoveController)