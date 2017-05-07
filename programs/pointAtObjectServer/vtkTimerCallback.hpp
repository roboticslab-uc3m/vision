#ifndef __VTK_TIMER_CALLBACK_HPP__
#define __VTK_TIMER_CALLBACK_HPP__

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>  // too slow
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>  // for file writer
#include <pcl/io/openni_grabber.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/surface/gp3.h>  // better use poisson; this leaves holes
#include <pcl/surface/poisson.h>
//#include <pcl/surface/vtk_utils.h> // worked in pcl-1.6
#include <pcl/surface/vtk_smoothing/vtk_utils.h> // worked in pcl-1.7

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkOBBTree.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVersion.h>

#include "SharedArea.hpp"

namespace roboticslab {

/**
 * @ingroup vtkTimerCallback
 *
 * The vtkTimerCallback object connects to the Kinect, connects its timer event to a member as a
 * callback function so the Kinect data can always be updated. It forces re-render, and recalculates
 * kinect object - Line intersection. The Line data is always updated as it proceeds from the Shared
 * Area.
 */
class vtkTimerCallback : public vtkCommand {
    public:
        static vtkTimerCallback *New() {
            vtkTimerCallback *cb = new vtkTimerCallback;
            // cb->TimerCount = 0;
            return cb;
        }
        void init();
        void setRenderer(vtkRenderer* _renderer);
        void setSharedArea(SharedArea* _sharedArea);
        void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cbCloud);  // The cloud callback
        virtual void Execute(vtkObject *caller, unsigned long eventId, void * vtkNotUsed(callData));

    private:
        // int TimerCount;
        // pcl::PCDWriter writer;
 
        vtkRenderer *renderer;
        SharedArea* sharedArea;

        void makeLineActor(vtkActor* _lineActor);
        vtkSmartPointer<vtkLineSource> lineSource;  // Keep here: he gets updated
        double p0[3];
        double p1[3];
        void updateLine();

        pcl::Grabber* interface;  // the Kinect Grabber
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // The updated Kinect cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        bool cloudInit;  // true if we have received first Point Cloud
        vtkSmartPointer<vtkPolyData> cloud_vtkPD;
        void makeCloudActor(vtkActor* _cloudActor);
        void updateCloud();
        vtkSmartPointer<vtkPolyData> cloud_filtered_vtkPD;
        void makeFilteredCloudActor(vtkActor* _filteredCloudActor);
        void updateFilteredCloud();

        void createFilteredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& _outCloud);

        void removePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr& _inOutCloud);

        vtkSmartPointer<vtkActorCollection> objectActorCollection;

        void objectSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& _inCloud, vtkActorCollection* _actorCollection);

        // Copied from pcl_visualizer.cpp @ 36441 
        void convertPointCloudToVTKPolyData (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                             vtkSmartPointer<vtkPolyData> &polydata);

};

}  // namespace roboticslab

#endif  // __VTK_TIMER_CALLBACK_HPP__

