#ifndef THREEDRPUI_H
#define THREEDRPUI_H

#include <QMainWindow>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <QVTKInteractor.h>
#include <vtkInteractorStyle.h>

QT_BEGIN_NAMESPACE
namespace Ui { class ThreeDRPUI; }
QT_END_NAMESPACE

class ThreeDRPUI : public QMainWindow
{
    Q_OBJECT

public:
    ThreeDRPUI(QWidget* parent = nullptr);
    ~ThreeDRPUI();

public slots:
    void onDrawSphereClick();

private:
    Ui::ThreeDRPUI* ui;

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> mRenderWindow;
    vtkSmartPointer<vtkRenderer> mRenderer;
    vtkSmartPointer<QVTKInteractor> mInteractor;
    vtkSmartPointer<vtkInteractorStyle> mInteractorStyle;
};
#endif // THREEDRPUI_H
