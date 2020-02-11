
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  //viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
  viewer.load_meshes_from_file("configuration.txt");
  viewer.initEdges();
  //viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/bunny.off");
  Init(*disp);
  //viewer.MyScale(Eigen::Vector3f(0.004, 0.02, 0.004));
  viewer.MyScale(Eigen::Vector3f(0.08, 0.08, 0.08));
  viewer.MyTranslate(Eigen::Vector3f(0, -0.6, 0));
  renderer.init(&viewer);
  //renderer.MultipleViews();
  
  renderer.line_less();

  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
