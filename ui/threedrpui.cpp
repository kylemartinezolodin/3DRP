#include "threedrpui.h"
#include "ui_threedrpui.h"

#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>

ThreeDRPUI::ThreeDRPUI(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::ThreeDRPUI),
    mRenderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
    mRenderer(vtkSmartPointer<vtkRenderer>::New()),
    mInteractor(vtkSmartPointer<QVTKInteractor>::New()),
    mInteractorStyle(vtkSmartPointer<vtkInteractorStyle>::New())
{
    ui->setupUi(this);
    
	// Set up the rendering
	mRenderWindow->AddRenderer(mRenderer);
	mRenderWindow->SetInteractor(mInteractor);
	ui->openGLWidget->SetRenderWindow(mRenderWindow);
	mInteractor->SetInteractorStyle(mInteractorStyle);
	mInteractor->Initialize();

	// Set the background color 
	mRenderer->SetBackground(1, 0, 0);

	// Set the UI connections
	QObject::connect(ui->actionOpen, &QAction::triggered,
		this, &ThreeDRPUI::onDrawSphereClick);
}

ThreeDRPUI::~ThreeDRPUI()
{
    delete ui;
}

void ThreeDRPUI::onDrawSphereClick() {
	// Create sphere
	vtkSmartPointer<vtkSphereSource> sphereSource =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetRadius(5);
	sphereSource->Update();

	// Create the actor where the sphere is rendered
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputData(sphereSource->GetOutput());

	vtkSmartPointer<vtkActor> sphere = vtkSmartPointer<vtkActor>::New();
	sphere->SetMapper(sphereMapper);

	// Add the sphere actor to the OpenGL
	mRenderer->AddViewProp(sphere);
	mRenderer->ResetCamera();
	mRenderWindow->Render();
}
