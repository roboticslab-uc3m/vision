#ifndef __POINT_AT_OBJECT_SERVER_HPP__
#define __POINT_AT_OBJECT_SERVER_HPP__

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAxesActor.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkImageGridSource.h>
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
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkVersion.h>

#include "SharedArea.hpp"
#include "LineCallbackPort.hpp"
#include "vtkTimerCallback.hpp"

namespace roboticslab {

/**
 * @ingroup PointAtObjectServer
 *
 * PointAtObjectServer is the main class. PointAtObjectServer creates:
 * - A VTK Renderer and VTK Render Window, adding axes and floor actors to it for spatial reference.
 * - A LineCallbackPort object (LineCallbackPort implements a callback on port reception of point data
 * to update the Line object).
 * - A vtkTimerCallback object (vtkTimerCallback connects to the Kinect, connects its timer event to a
 * member as a callback function so the Kinect data can always be updated; it forces re-render, and
 * recalculates kinect object - Line intersection).
 * - A SharedArea through where the Line object information is safely shared (internal use of semaphores).
 */
class PointAtObjectServer {
    protected:
        LineCallbackPort lineCallbackPort;
        vtkSmartPointer<vtkTimerCallback> timerCallback;
        SharedArea sharedArea;

        void makeFloorActor(vtkActor* _floorActor);

    public:
        PointAtObjectServer() { }
        bool init();
        bool close();
};

}  // namespace roboticslab

#endif  // __POINT_AT_OBJECT_SERVER_HPP__

