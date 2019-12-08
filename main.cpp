
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"





int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  //viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
  viewer.load_meshes_from_file("configuration.txt");
  //viewer.load_mesh_from_file("C:\\Users\\Guyp\\Documents\\Ass3D\\p1\\tutorial\\data\\cube.obj");
  //viewer.load_mesh_from_file("C:\\Users\\Guyp\\Documents\\Ass3D\\p1\\tutorial\\data\\bunny.off");
  viewer.initEdges2();
  Init(*disp);
  renderer.init(&viewer);
  
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
