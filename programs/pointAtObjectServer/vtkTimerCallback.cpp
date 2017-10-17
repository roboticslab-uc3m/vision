#include "vtkTimerCallback.hpp"

#include <iostream>

namespace roboticslab {

/************************************************************************/
void vtkTimerCallback::setSharedArea(SharedArea* _sharedArea) {
    sharedArea = _sharedArea;
}

/************************************************************************/
void vtkTimerCallback::setRenderer(vtkRenderer* _renderer) {
    renderer = _renderer;
}

/************************************************************************/
void vtkTimerCallback::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cbCloud) {
    // printf("[vtkTimerCallback] inside cloud_cb_()\n"); //
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cbCloud, *cloud);  // source, dest.
    cloudInit=true;
}

/************************************************************************/
void vtkTimerCallback::init() {

    // Make some room for the Kinect Cloud and Filtered Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = tmpCloud;  // tmpCloud.reset(); needed? dangerous?
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered = tmpCloud2;  // tmpCloud2.reset(); needed? dangerous?
    cloudInit = false;

    // Connect to the Kinect and set the callback for always having Kinect cloud updated
    printf("Waiting to connect to Kinect... (warning: no timeout implemented yet)\n");
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&vtkTimerCallback::cloud_cb_, this, _1);
    interface->registerCallback (f);
    interface->start();
    while(!cloudInit);  // dangerous! waiting for first Point Cloud reception
    printf("Connected to Kinect!\n");

    // Let's stop the Kinect while we prepare stuff
    // interface->stop();

    // Create Line and add the actor to the renderer
    vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
    makeLineActor(lineActor);
    renderer->AddActor(lineActor);

    //vtkSmartPointer<vtkActor> cloudActor = vtkSmartPointer<vtkActor>::New();
    //makeCloudActor(cloudActor);
    //renderer->AddActor(cloudActor);

    vtkSmartPointer<vtkActor> filteredCloudActor = vtkSmartPointer<vtkActor>::New();
    makeFilteredCloudActor(filteredCloudActor);
    renderer->AddActor(filteredCloudActor);

    objectActorCollection = vtkSmartPointer<vtkActorCollection>::New();

    // -- This would load Point Cloud from file: --
    // sensor_msgs::PointCloud2 cloud_blob;
    // pcl::io::loadPCDFile ("../testArm.pcd", cloud_blob);
    // pcl::fromROSMsg (cloud_blob, *cloud);
    // -- This would write Point Cloud to file: --
    // writer.write<pcl::PointXYZ> ("cloud.pcd", *cloud, false);

    // Filter the Cloud before aything else
    //createFilteredCloud(cloud_filtered);

    // Create Filtered Cloud actor and add the actor to the renderer
    //vtkSmartPointer<vtkActor> filteredCloudActor = vtkSmartPointer<vtkActor>::New();
    //makeCloudActor(cloud_filtered, filteredCloudActor);
    //renderer->AddActor(filteredCloudActor);

    // Remove the planes form this Filtered Cloud
    //removePlanes(cloud_filtered);

    // update the Line
    //updateLine();

    // Make an Actor collection, fill it with objects, render the actors
    /*objectSegmentation(cloud_filtered, objectActorCollection);
    objectActorCollection->InitTraversal();  // Does something like moving iterator to begin element
    for(vtkIdType i = 0; i < objectActorCollection->GetNumberOfItems(); i++) {
        vtkSmartPointer<vtkActor> nextActor = objectActorCollection->GetNextActor();
        renderer->AddActor(nextActor);
        printf("Rendered object actor %d.\n",i);
    }*/
}

/************************************************************************/
void vtkTimerCallback::Execute(vtkObject *caller, unsigned long eventId, void * vtkNotUsed(callData)) {
//    interface->start ();
//    interface->stop ();

    // if (vtkCommand::TimerEvent == eventId) { ++this->TimerCount; }
    // std::cout << this->TimerCount << std::endl;

    updateLine();
    //updateCloud();
    createFilteredCloud(cloud_filtered);
    updateFilteredCloud();
    removePlanes(cloud_filtered);
    objectSegmentation(cloud_filtered, objectActorCollection);

    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();

}

/************************************************************************/
void vtkTimerCallback::makeLineActor(vtkActor* _lineActor) {
    // Just for starters, later we will call updateLine
    double pi0[3] = {0, 0, 0};
    double pi1[3] = {0, 0, 0};

    // Create a graphical representation of the pointing line
    lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(p0);
    lineSource->SetPoint2(p1);
    lineSource->Update();

    // Map to graphics library
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

#if VTK_MAJOR_VERSION <= 5
    lineMapper->SetInput(lineSource->GetOutput());
#else
    lineMapper->SetInputConnection(lineSource->GetOutputPort());
#endif

    // Actor coordinates geometry, properties, transformation
    _lineActor->SetMapper(lineMapper);
    _lineActor->GetProperty()->SetColor(1,1,1);  // Line color white
}

/************************************************************************/
void vtkTimerCallback::updateLine() {
    // Get the last updated line point data
    double lineCoords[6];
    //BEGIN_CRITICAL//j//sharedArea->getLC(lineCoords);
    sharedArea->getLongLC(lineCoords);//j//END_CRITICAL//
    p0[0] = lineCoords[0]; p0[1] = lineCoords[1]; p0[2] = lineCoords[2];
    p1[0] = lineCoords[3]; p1[1] = lineCoords[4]; p1[2] = lineCoords[5];
    lineSource->SetPoint1(p0);
    lineSource->SetPoint2(p1);
    lineSource->Update();
}

/************************************************************************/
void vtkTimerCallback::makeCloudActor(vtkActor* _cloudActor) {
    cloud_vtkPD = vtkSmartPointer<vtkPolyData>::New();
    convertPointCloudToVTKPolyData ( *cloud, cloud_vtkPD);
    vtkSmartPointer<vtkPolyDataMapper> cloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    cloudMapper->SetInput(cloud_vtkPD);
#else
    cloudMapper->SetInputData(cloud_vtkPD);
#endif
    _cloudActor->SetMapper(cloudMapper);
    _cloudActor->GetProperty()->SetColor(1,1,1);  // Cloud color white
}

/************************************************************************/
void vtkTimerCallback::makeFilteredCloudActor(vtkActor* _filteredCloudActor) {
    cloud_filtered_vtkPD = vtkSmartPointer<vtkPolyData>::New();
    convertPointCloudToVTKPolyData ( *cloud_filtered, cloud_filtered_vtkPD);
    vtkSmartPointer<vtkPolyDataMapper> filteredCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    filteredCloudMapper->SetInput(cloud_filtered_vtkPD);
#else
    filteredCloudMapper->SetInputData(cloud_filtered_vtkPD);
#endif
    _filteredCloudActor->SetMapper(filteredCloudMapper);
    _filteredCloudActor->GetProperty()->SetColor(1,1,1);  // Cloud color white
}

/************************************************************************/
void vtkTimerCallback::updateCloud() {
    convertPointCloudToVTKPolyData ( *cloud, cloud_vtkPD);
}

/************************************************************************/
void vtkTimerCallback::updateFilteredCloud() {
    convertPointCloudToVTKPolyData ( *cloud_filtered, cloud_filtered_vtkPD);
}

/************************************************************************/
void vtkTimerCallback::createFilteredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& _outCloud) {
    //std::cout << "Starting to remove NaNs from PointCloud of " << cloud->points.size() << " points..." << endl;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*cloud, *tmp, indices);
    //std::cout << "Starting to voxelize PointCloud of " << _outCloud->points.size() << " points for filtering..." << endl;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (tmp);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*_outCloud);
    //std::cout << "Done voxelize, now have PointCloud of " << _outCloud->points.size() << " points..." << endl;
}

/************************************************************************/
void vtkTimerCallback::removePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr& _inOutCloud) {
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    
    int i=0, nr_points = (int) _inOutCloud->points.size ();
    while (_inOutCloud->points.size () > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (_inOutCloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (_inOutCloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane);
        // std::cout << "PointCloud representing the planar component: "
        //           << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        _inOutCloud = cloud_f;
    }
    //std::cout << "Left with PointCloud of " << _inOutCloud->points.size() << " points after remove planes..." << endl;
}

/************************************************************************/
void vtkTimerCallback::objectSegmentation( pcl::PointCloud<pcl::PointXYZ>::Ptr& _inCloud,
                                           vtkActorCollection* _actorCollection) {

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (_inCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.025); // 2cm
    ec.setMinClusterSize (25);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (_inCloud);
    ec.extract (cluster_indices);
    // std::cout << "Euclidean Cluster extraction successful..." << std::endl;

//    renderer->RemoveActor(_actorCollection->GetLastActor()); 
    _actorCollection->InitTraversal();  // Does something like moving iterator to begin element
    for(vtkIdType i = 0; i < _actorCollection->GetNumberOfItems(); i++) {
        //vtkSmartPointer<vtkActor> nextActor = _actorCollection->GetNextActor();
        //renderer->RemoveActor(nextActor);
        renderer->RemoveActor(_actorCollection->GetNextActor());
        //printf("Removed object actor %d.\n",i);
    }
    _actorCollection->RemoveAllItems();
    _actorCollection->InitTraversal();  // Does something like moving iterator to begin element
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (_inCloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        j++;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
        //std::cout << "Cluster " << j << " is a " << cloud_cluster->points.size() << " point PointCloud." << std::endl;

        // Normal estimation (normals should not contain the point normals + surface curvatures)
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_cluster);
        n.setInputCloud (cloud_cluster);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        //n.setKSearch (cloud_cluster->size());
        n.compute (*normals);
        // Concatenate the XYZ and normal fields (cloud_with_normals = cloud + normals)
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud_cluster, *normals, *cloud_with_normals);
        // Create search tree
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        pcl::Poisson<pcl::PointNormal> psr;  // Only pcl >= 1.6
        pcl::PolygonMesh triangles;
        psr.setInputCloud(cloud_with_normals);
        psr.setSearchMethod(tree2);
        psr.reconstruct(triangles);

        ////////////////////////////////////////////////////////////////////////
        /*
        // pcl 1.5 users can use this previously implemented method instead:
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;
        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (1);
        // Set typical values for the parameters
        gp3.setMu (2.5);
        // gp3.setMaximumNearestNeighbors (100);
        gp3.setMaximumNearestNeighbors (cloud_cluster->size());
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(0); // M_PI/18, 10 degrees
        gp3.setMaximumAngle(M_PI); // 2*M_PI/3, 120 degrees
        gp3.setNormalConsistency(false);
        // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);
        */
        
        // Create the kinect-obtained object
        vtkSmartPointer<vtkPolyData> object = vtkSmartPointer<vtkPolyData>::New();
        // Pass the surface mesh to the kinect-obtained object
        pcl::VTKUtils::convertToVTK(triangles,object);

        ///////////////////////////////////////////////////////////////////////
    
        // Create the locator
        vtkSmartPointer<vtkOBBTree> otree = vtkSmartPointer<vtkOBBTree>::New();
        otree->SetDataSet(object);
        otree->BuildLocator();

        // Intersect the locator with the line
        vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
        otree->IntersectWithLine(p0, p1, intersectPoints, NULL);

        if(intersectPoints->GetNumberOfPoints() > 0) {
            std::cout << "Intersection of Line with Object_" << j << " at "
            << intersectPoints->GetNumberOfPoints() << " different point(s)." << std::endl;
        }

        for(int i=0;i<intersectPoints->GetNumberOfPoints();i++) {
            double intersection[3];
            intersectPoints->GetPoint(i, intersection);
            std::cout << "Intersection: "  << intersection[0] << ", " 
                      << intersection[1] << ", " << intersection[2] << std::endl;
        }
        ///////////////////////////////////////////////////////////////////////

        // [object] map to graphics library
        vtkSmartPointer<vtkPolyDataMapper> objectMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
        objectMapper->SetInput(object);
#else
        objectMapper->SetInputData(object);
#endif

        // [object] actor coordinates geometry, properties, transformation
        vtkSmartPointer<vtkActor> objectActor = vtkSmartPointer<vtkActor>::New();
        objectActor->SetMapper(objectMapper);
        if(intersectPoints->GetNumberOfPoints() > 0) objectActor->GetProperty()->SetColor(1,0,0); // color red
        else objectActor->GetProperty()->SetColor(0,0,1); // color blue

        // add the actor to the scene
        //renderer->AddActor(objectActor);
        _actorCollection->AddItem(objectActor);
        //objectActor->Delete();
    }
    _actorCollection->InitTraversal();  // Does something like moving iterator to begin element
    for(vtkIdType i = 0; i < _actorCollection->GetNumberOfItems(); i++) {
        vtkSmartPointer<vtkActor> nextActor = _actorCollection->GetNextActor();
        renderer->AddActor(nextActor);
        //printf("Rendered object actor %d.\n",i);
    }
}

/************************************************************************/
void vtkTimerCallback::convertPointCloudToVTKPolyData (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                                                  vtkSmartPointer<vtkPolyData> &polydata) {
    if (!polydata) polydata = vtkSmartPointer<vtkPolyData>::New ();

    // Create the supporting structures
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPoints> points      = vtkSmartPointer<vtkPoints>::New ();

    // Set the points
    points->SetDataTypeToFloat ();
    points->SetNumberOfPoints (cloud.points.size ());
    double p[3];

    for (vtkIdType i = 0; i < (int)cloud.points.size (); ++i) {
        p[0] = cloud.points[i].x;
        p[1] = cloud.points[i].y;
        p[2] = cloud.points[i].z;
        points->SetPoint (i, p);
        vertices->InsertNextCell ((vtkIdType)1, &i);
    }
    polydata->SetPoints (points);
    polydata->SetVerts (vertices);
}

/************************************************************************/
}  // namespace roboticslab

