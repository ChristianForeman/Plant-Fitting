#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.height = 1;
    cloud.width = 100 * 100 * 100;
    cloud.points.reserve(100 * 100 * 100);

    // Fill in the cloud data

    pcl::PointXYZ pt;

    for(double i = -1; i < 1; i += 0.02) {
        for(double j = -1; j < 1; j += 0.02) {
            for(double k = -1; k < 1; k += 0.02) {
                pt.x = i;
                pt.y = j;
                pt.z = k;

                cloud.points.push_back(pt);
            }
        }
    }

    pcl::io::savePCDFileASCII ("pcs/pc_grid.pcd", cloud);

    return (0);
}