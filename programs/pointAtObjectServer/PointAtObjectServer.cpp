#include "PointAtObjectServer.hpp"

namespace roboticslab {

/************************************************************************/
bool PointAtObjectServer::init() {
    sharedArea.init();
    lineCallbackPort.setSharedArea(&sharedArea);

    // Create a VTK Renderer and Render Window
    vtkRenderer *renderer = vtkRenderer::New();
    vtkRenderWindow *renderWindow = vtkRenderWindow::New();
    renderWindow->AddRenderer(renderer);

    // Create a VTK Interactor
    vtkRenderWindowInteractor *renderWindowInteractor = vtkRenderWindowInteractor::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Create axes and add the actor to the renderer
    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    axesActor->SetTotalLength(0.1,0.1,0.1);
    axesActor->AxisLabelsOff();
    renderer->AddActor(axesActor);

    // Create floor and add the actor to the renderer
    vtkSmartPointer<vtkActor> floorActor = vtkSmartPointer<vtkActor>::New();
    makeFloorActor(floorActor);
    renderer->AddActor(floorActor);

    // Background color black
    renderer->SetBackground(0,0,0);

    // Render the image (lights and cameras are created automatically)
    renderWindow->Render();

    // Initialize must be called prior to creating timer events
    renderWindowInteractor->Initialize();

    // Sign up to receive TimerEvent
    timerCallback = vtkSmartPointer<vtkTimerCallback>::New();
    timerCallback->setRenderer(renderer);
    timerCallback->setSharedArea(&sharedArea);
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    timerCallback->init();
    int timerId = renderWindowInteractor->CreateRepeatingTimer(1000);  // [ms]
    std::cout << "timerId: " << timerId << std::endl;

    // Configure and open lineCallbackPort
    lineCallbackPort.open("/pointAtObject:i");
    lineCallbackPort.useCallback();

    // begin mouse interaction (blocking)
    renderWindowInteractor->Start();

    return true;
}

/************************************************************************/
bool PointAtObjectServer::close() {
    printf("[PointAtObjectServer] closing...\n");
    lineCallbackPort.disableCallback();
    lineCallbackPort.interrupt();
    printf("[PointAtObjectServer] Bye!\n");
    return true;
}

/************************************************************************/

void PointAtObjectServer::makeFloorActor(vtkActor* _floorActor) {
    // Create a polyline for a 1x1 square (floor)
    double p0[3] = {0.0, 0.0, 0.0};
    double p1[3] = {0.0, 1.0, 0.0};
    double p2[3] = {1.0, 1.0, 0.0};
    double p3[3] = {1.0, 0.0, 0.0};
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(p0);
    points->InsertNextPoint(p1);
    points->InsertNextPoint(p2);
    points->InsertNextPoint(p3);
    points->InsertNextPoint(p0);
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for(unsigned int i = 0; i < 4; i++) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0,i);
        line->GetPointIds()->SetId(1,i+1);
        lines->InsertNextCell(line);
    }
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    linesPolyData->SetPoints(points);
    linesPolyData->SetLines(lines);

    // Maps to graphics library
    vtkPolyDataMapper *floorMapper = vtkPolyDataMapper::New();
#if VTK_MAJOR_VERSION <= 5
    floorMapper->SetInput(linesPolyData);
#else
    floorMapper->SetInputData(linesPolyData);
#endif

    // Actor coordinates geometry, properties, transformation
    _floorActor->SetMapper(floorMapper);
    _floorActor->GetProperty()->SetColor(0.5,0.5,0.5); // color
}

/************************************************************************/
}  // namespace roboticslab


