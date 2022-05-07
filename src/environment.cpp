#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// Function Name: initHighway
// Function Description: Initialize the 3D scene with highway constructed from egoCar (Host Car) and surrounding cars
// Inputs:
// - renderScene : whether to render the highwayscene or not
// - viewer: pointer to the PCL visualizer
// Outputs: vector of the constructed cars

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // Construction of each car in the scene using the following constructor
    // Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
    // Each of the inputs is a vector of dimention 1x3.
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    // Constructing a vector of all the constructed cars.
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// Function Name: simpleHighway
// Function Description: Open 3D viewer and display simple highway
// Inputs:
// - viewer: pointer to the PCL visualizer
// Outputs: Non

////commented for city block to run////
/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_clusters = true;
    bool render_box = true;
    bool render_obs = true;
    bool render_plane = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar lidar = Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar.scan();

    // renderRays(viewer, lidar.position, cloud);
    // renderPointCloud(viewer, cloud, "vvvv", Color(1,1,0));
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> PPC;

    // Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = PPC.SegmentRansacPlane(cloud, 100, 0.2);
    if (render_obs)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PPC.Clustering_kdtree(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size ";
            PPC.numPoints(cluster);

            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box)
        {
            Box box = PPC.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
    // rendering
    // Bounding Box
}
*/

// Function Name: initCamera
// Function Description: adjust the camera position in the scene
// Inputs:
// - setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Sideview, FPS}
// - viewer: pointer to the PCL visualizer
// Outputs: Non

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    // XY gives a 45 degree angle view,
    // while FPS is First Person Sense and gives the sensation of being in the car’s driver seat.
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

// function used rendering the city block , segmenting , clustering and drawing boundingboxes
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool render_clusters = true;
    bool render_box = true;
    bool render_obs = false;
    bool render_plane = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.4, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
    // Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentRansacPlane(filterCloud, 40, 0.3);
    if (render_obs)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_kdtree(segmentCloud.first, 0.5, 10, 140);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 1), Color(0, 0, 1), Color(1, 1, 0), Color(1, 1, 1), Color(1, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size ";

            pointProcessorI->numPoints(cluster);

            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}
// Function Name: Main Function
// Function Description: Initialize viewer - Initialize camera position - Plot the highway scene

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}