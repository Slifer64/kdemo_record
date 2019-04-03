#include <kdemo_record/kdemo_record.h>

#include <QApplication>
#include <memory>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kdemo_record");

    std::shared_ptr<KDemoRecord> kdemo_rec;
    kdemo_rec.reset(new KDemoRecord);
    kdemo_rec->run();

    return 0;
}
