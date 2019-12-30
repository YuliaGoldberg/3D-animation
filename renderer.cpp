#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;

}

IGL_INLINE void Renderer::draw(GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}


	for (auto& core : core_list)
	{

		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id)
			{
				Eigen::Matrix4f cal = scn->MakeTrans() * scn->CalcParentsTrans(mesh.id);
				core.draw(cal, mesh);

			}

		}




	}
}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	
	if (button == 1)
	{
		if ((int)scn->selected_data_index > 1) {
			scn->data_list[1].MyTranslate(Eigen::Vector3f(-xrel / 200.0f, 0, 0));
			scn->data_list[1].MyTranslate(Eigen::Vector3f(0, yrel / 200.0f, 0));
		}
		else if (scn->selected_data_index ==0|| scn->selected_data_index == 1) {
			scn->data().MyTranslate(Eigen::Vector3f(-xrel / 200.0f, 0, 0));
			scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 200.0f, 0));
		}
		else {
			scn->MyTranslate(Eigen::Vector3f(-xrel / 200.0f, 0, 0));
			scn->MyTranslate(Eigen::Vector3f(0, yrel / 200.0f, 0));
		}
		
	}
	else
	{
		if (scn->selected_data_index == -1) {
			scn->MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		
		}
		else {
			scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		}
	}
	
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

double Renderer::Picking2(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() *scn->CalcParentsTrans(scn->selected_data_index) *scn->data().MakeTrans() ;
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
		{
			int p1, p2, p3;
			p1 = scn->data().F.row(fid)[0];
			p2 = scn->data().F.row(fid)[1];
			p3 = scn->data().F.row(fid)[2];
			Eigen::Vector3f v1(scn->data().V.row(p1)[0], scn->data().V.row(p1)[1], scn->data().V.row(p1)[2]);
			Eigen::Vector3f v2(scn->data().V.row(p2)[0], scn->data().V.row(p2)[1], scn->data().V.row(p2)[2]);
			Eigen::Vector3f v3(scn->data().V.row(p3)[0], scn->data().V.row(p3)[1], scn->data().V.row(p3)[2]);
			Eigen::Vector3f b(v1[0]*bc[0]+v2[0]*bc[1]+v3[0]*bc[2], v1[1] * bc[0] + v2[1] * bc[1] + v3[1] * bc[2], v1[2] * bc[0] + v2[2] * bc[1] + v3[2] * bc[2]);
			Eigen::Vector4f a(4);
			a << b, 1;
			Eigen::Vector4f ans4f(4);
			ans4f = view * a;
			return ans4f.norm();
		}
		return INFINITY;
}

//previous function
bool Renderer::Picking(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() * scn->data().MakeTrans();
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
	{
		return true;
	}
	return false;

}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}
	void Renderer::animate(GLFWwindow* window) {
		
	}
	Eigen::Vector3f Renderer::Calc_E() {
		int index = scn->data_list.size() - 1;
		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(scn->data_list.size());
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(scn->data_list[index].V.colwise().mean()[0],
			scn->data_list[index].V.colwise().maxCoeff()[1], scn->data_list[index].V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	Eigen::Vector3f Renderer::Calc_R(int index) {
		
		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(index+1);
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(scn->data_list[index].V.colwise().mean()[0],
			scn->data_list[index].V.colwise().minCoeff()[1], scn->data_list[index].V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	void Renderer::ik_solver() {
		
		
		Eigen::Vector3f D = scn->data_list[0].MakeTrans().col(3).head(3);
		Eigen::Vector3f Base = Calc_R(1);
		Eigen::Vector3f E = Calc_E();
		double EDdistance = (D - E).norm();
		double BaseDdistance = (D-Base).norm();
		double max_Arm_length = 1.6 * scn->data_list_indices.size();
		if (BaseDdistance > max_Arm_length) {
			cout << "cannot reach" << endl;
			scn->ik_flag = false;
			return;
		}
		if(EDdistance<0.1){
			scn->ik_flag = false;
			return;
		}


			for (int i = scn->data_list.size() - 1; i > 0; i--) {
				
				 
				Eigen::Vector3f R = Calc_R(i);
				
				Eigen::Vector3f RD = (D - R).normalized();
				Eigen::Vector3f RE = (E - R).normalized();
				float pre_angle = RE.dot(RD);
				if (pre_angle > 1) {
					pre_angle = 1;
				}
				else if (pre_angle < -1)
					pre_angle = -1;

				float angle = acosf(pre_angle);

				Eigen::Vector3f cross = RE.cross(RD).normalized();
				Eigen::Vector3f crossInverse = scn->CalcParentsInverse(scn->data_list.size()) * cross;
				if (EDdistance > 0.5)
					angle = angle / 10;
				scn->data_list[i].MyRotate(crossInverse, angle);
				E = Calc_E();
				EDdistance = (D - E).norm();
			}
		

	}
	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}