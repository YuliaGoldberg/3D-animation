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

IGL_INLINE void Renderer::draw( GLFWwindow* window)
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
		post_resize(window,width, height);
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
				Eigen::Matrix4f cal = scn->MakeTrans() * scn->CalcParentsTrans(mesh.prev);
				core.draw(cal, mesh,true);
			}

		}
	}

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}
void Renderer::MultipleViews()
{
	unsigned int left_view, right_view;
	core().viewport = Eigen::Vector4f(0, 0, 640, 800);
	left_view = core_list[0].id;
	right_view = append_core(Eigen::Vector4f(640, 0, 640, 800));
	scn->snake_head->set_visible(true, left_view);
	scn->data_list[10].set_visible(true, right_view);
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	int w = mode->width, h = mode->height;
	core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
	core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);


}
IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{

	scn = viewer;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);
	//****splite screen
	core().viewport = Eigen::Vector4f(0, 0, 500, 800);
	left_view = core_list[0].id;
	right_view = append_core(Eigen::Vector4f(500, 0, 500, 800));
	locateCore1();
}
void Renderer::locateCore1()
{
	Eigen::RowVector3f N = scn->data(9).GetRotation().matrix() * Eigen::Vector3f(0, 0, 1);
	core(right_view).camera_up = N;
	Eigen::RowVector3f E = scn->data(9).GetRotation().matrix() * Eigen::Vector3f(0, -2.09, 0);
	core(right_view).camera_eye = E;

	Eigen::RowVector3f TR = -(scn->MakeTrans() * scn->CalcParentsTrans(scn->snake_head)).col(3).head(3);//A.col(1);
	//TR = TR + Eigen::RowVector3f(0, -((0.91) + (9 * 1.6)), 0);
	Eigen::Vector3f a = (scn->CalParentsRotationMatrixes(scn->snake_head).matrix().col(1) * -0.83);


	TR += a;
	core(right_view).camera_translation = TR;
	for (int i = 0; i<scn->data_list.size(); ++i)
	{
		scn->data_list[i].set_visible(true, left_view);
		scn->data_list[i].set_visible(true, right_view);
	}
}


void Renderer::UpdateCore() {

	Eigen::Matrix3f rota = scn->CalParentsRotationMatrixes(scn->snake_head);
	// TODO - play with yTheta in order to fix the camera_eye
	core(right_view).camera_up = rota * Eigen::Vector3f(0, 0, 1);
	core(right_view).camera_eye = rota * Eigen::Vector3f(0, -2.09, 0);
	Eigen::RowVector3f TR = -(scn->MakeTrans() * scn->CalcParentsTrans(scn->snake_head)).col(3).head(3); //A.col(1);
	Eigen::Vector3f a = (scn->CalParentsRotationMatrixes(scn->snake_head).matrix().col(1) * -0.83);
		

	TR += a;

	core(right_view).camera_translation = TR;

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
		if (scn->picked_mesh) {
			if (scn->data().prev != nullptr)
			{
				scn->data(0).MyTranslate(Eigen::Vector3f(-xrel / 20.0f, 0, 0));
				scn->data(0).MyTranslate(Eigen::Vector3f(0, yrel / 20.0f, 0));
			}
			else {
				scn->data().MyTranslate(Eigen::Vector3f(-xrel / 20.0f, 0, 0));
				scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 20.0f, 0));
			}
			//****splite screen
			
			UpdateCore();
		}
	
		
	}
	else
	{
		if (scn->picked_mesh) {
			scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
			//****splite screen
			ik_fixer();
			UpdateCore();
			
			
		}
		else {
			scn->MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
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
	double y = core(left_view).viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core(left_view).camera_eye, core(left_view).camera_center, core(left_view).camera_up, view);
	view = view * (core(left_view).trackball_angle * Eigen::Scaling(core(left_view).camera_zoom * core(left_view).camera_base_zoom)
		* Eigen::Translation3f(core(left_view).camera_translation + core(left_view).camera_base_translation)).matrix() * scn->MakeTrans() * scn->CalcParentsTrans(scn->data().prev) * scn->data().MakeTrans();
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core(left_view).proj, core(left_view).viewport, scn->data().V, scn->data().F, fid, bc))
	{
		Eigen::MatrixXi F = scn->data().F;
		Eigen::MatrixXd V = scn->data().V;

		//Vector4f v5 = Vector4f (1.0 f , 2.0 f , 3.0 f , 4.0 f ) ;

		Eigen::Vector3f v0, v1, v2, p;
		v0 = V.row(F.row(fid)(0)).cast<float>();
		v1 = V.row(F.row(fid)(1)).cast<float>();
		v2 = V.row(F.row(fid)(2)).cast<float>();

		Eigen::Vector4f u0, u1, u2;
		u0 = Eigen::Vector4f(v0[0], v0[1], v0[2], 1.0f);
		u1 = Eigen::Vector4f(v1[0], v1[1], v1[2], 1.0f);
		u2 = Eigen::Vector4f(v2[0], v2[1], v2[2], 1.0f);

		u0 = view * u0;
		u1 = view * u1;
		u2 = view * u2;

		v0 = Eigen::Vector3f(u0[0], u0[1], u0[2]);
		v1 = Eigen::Vector3f(u1[0], u1[1], u1[2]);
		v2 = Eigen::Vector3f(u2[0], u2[1], u2[2]);

		p = v0 * bc[0] + v1 * bc[1] + v2 * bc[2];
		return p[2];
	}
	return INFINITY;
}

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
			core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
			core(right_view).viewport = Eigen::Vector4f(w/2, 0, w / 2, h);
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
	void Renderer::line_less() {
		for (int i = 0; i < scn->data_list.size(); ++i) {
			core(left_view).toggle(scn->data_list[i].show_lines);
			//core(right_view).toggle(scn->data_list[i].show_lines);
			core(right_view).toggle(scn->data_list[i].show_faces);
		}
	}
	Eigen::Vector3f Renderer::Calc_E() {
		
		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(scn->snake_head);
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(scn->snake_head->V.colwise().mean()[0],
			scn->snake_head->V.colwise().maxCoeff()[1], scn->snake_head->V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	Eigen::Vector3f Renderer::Calc_R(igl::opengl::ViewerData * ptr) {

		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(ptr);
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(ptr->V.colwise().mean()[0],
			ptr->V.colwise().minCoeff()[1], ptr->V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	void Renderer::ik_fixer() {
		for (int i = 0; i < 10; ++i)
		{
			Eigen::Matrix3f rot = scn->data_list[i].GetRotation();
			float y1 = 0;
			if (rot.row(1)[1] < 1 && rot.row(1)[1] > -1) {

				y1 = atan2f(rot.row(1)[0], -rot.row(1)[2]);
			}
			scn->data_list[i].MyRotate(Eigen::Vector3f(0, 1, 0), -y1);
			if (10 > i + 1) {
				scn->data_list[i + 1].RotInSys(Eigen::Vector3f(0, 1, 0), y1);
			}

		}
	}

	void Renderer::ik_solver() {
		Eigen::Vector3f D = scn->data_list[scn->selected_data_index].MakeTrans().col(3).head(3);
		Eigen::Vector3f Base = Calc_R(&scn->data_list[0]);
		Eigen::Vector3f E = Calc_E();
		double EDdistance = (D - E).norm();
		double BaseDdistance = (D - Base).norm();
		double max_Arm_length = 1.6 * 10;
		//if (BaseDdistance > max_Arm_length) {
		//	std::cout << "cannot reach" << std::endl;
		//	std::cout << "BaseDdistance: " << BaseDdistance << std::endl;

		//	//scn->ik_flag = false;
		//	ik_fixer();
		//	return;
		//}
		//if (/*EDdistance<0.1 */)){
		//	ik_fixer();
		//	scn->ik_flag = false;
		//	return;
		//}


		for (auto ptr = scn->snake_head; ptr!=nullptr; ptr=ptr->prev) {
			//****splite screen
			if (checkCollision_helper()) {
				scn->ik_flag = false;
			}
			UpdateCore();
			Eigen::Vector3f R = Calc_R(ptr);

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
			Eigen::Vector3f crossInverse = scn->CalcParentsInverse(scn->snake_head) * cross;
			if (EDdistance > 0.5)
				angle = angle / 20;
			ptr->MyRotate(crossInverse, angle);
			E = Calc_E();
		
		}


	}
	bool Renderer::checkCollision_helper()
	{
		Eigen::Matrix3f snake_rotation = scn->snake_head->GetRotation().matrix();
		Eigen::Matrix3f obj_rotation = scn->data().GetRotation().matrix();
		
		Eigen::Matrix3f C = snake_rotation.transpose() * obj_rotation;
		
		
		return checkCollision(&scn->snake_head->tree, &scn->data().tree,&snake_rotation,&obj_rotation,&C);

	}

	bool Renderer::checkCollision(igl::AABB<Eigen::MatrixXd, 3>* bunnyA, igl::AABB<Eigen::MatrixXd, 3>* bunnyB, Eigen::Matrix3f* A, Eigen::Matrix3f* B, Eigen::Matrix3f* C) {
		Eigen::Vector3f bunnyAcenter = bunnyA->m_box.center().cast<float>();
		Eigen::Vector3f bunnyBcenter = bunnyB->m_box.center().cast<float>();
		Eigen::Vector3f centerC0 = (scn->data().MakeTrans() * Eigen::Vector4f(bunnyAcenter[0],
			bunnyAcenter[1], bunnyAcenter[2], 1)).head(3);
		Eigen::Vector3f centerC1 = (scn->CalcParentsTrans(scn->snake_head) * Eigen::Vector4f(bunnyBcenter[0],
			bunnyBcenter[1], bunnyBcenter[2], 1)).head(3);
		Eigen::Vector3f D = centerC1 - centerC0;
		
		float a0 = bunnyA->m_box.sizes().cast<float>()[0] / 2;
		float a1 = bunnyA->m_box.sizes().cast<float>()[1] / 2;
		float a2 = bunnyA->m_box.sizes().cast<float>()[2] / 2;
		float b0 = bunnyB->m_box.sizes().cast<float>()[0] / 2;
		float b1 = bunnyB->m_box.sizes().cast<float>()[1] / 2;
		float b2 = bunnyB->m_box.sizes().cast<float>()[2] / 2;
		bool tableCalcAns = tableCalc(A, a0, a1, a2, B, b0, b1, b2, C, &D);
		if (!tableCalcAns)
		{
			if (bunnyA->is_leaf())
			{
				if (bunnyB->is_leaf())
				{
					scn->ik_flag = false;
					snake_win = true;
					return true;
				}
				else
				{
					return checkCollision(bunnyA, bunnyB->m_left, A, B, C) || checkCollision(bunnyA, bunnyB->m_right, A, B, C);
				}
			}
			else if (bunnyB->is_leaf())
			{
				return checkCollision(bunnyA->m_left, bunnyB, A, B, C) || checkCollision(bunnyA->m_right, bunnyB, A, B, C);
			}
			else
			{
				return checkCollision(bunnyA->m_left, bunnyB->m_left, A, B, C) || checkCollision(bunnyA->m_left, bunnyB->m_right, A, B, C) || checkCollision(bunnyA->m_right, bunnyB->m_left, A, B, C) || checkCollision(bunnyA->m_right, bunnyB->m_right, A, B, C);
			}

		}
		else//no Collision
		{
			return false;
		}
	}
	
	


	bool Renderer::tableCalc(Eigen::Matrix3f* A, float a0, float a1, float a2, Eigen::Matrix3f* B, float b0, float b1, float b2, Eigen::Matrix3f* C, Eigen::Vector3f* D) {
		return (a0 + (b0 * abs(C->row(0)[0]) + b1 * abs(C->row(0)[1]) + b2 * abs(C->row(0)[2])) < abs(A->col(0).dot(*D))) ||
			(a1 + (b0 * abs(C->row(1)[0]) + b1 * abs(C->row(1)[1]) + b2 * abs(C->row(1)[2])) < abs(A->col(1).dot(*D))) ||
			(a2 + (b0 * abs(C->row(2)[0]) + b1 * abs(C->row(2)[1]) + b2 * abs(C->row(2)[2])) < abs(A->col(2).dot(*D))) ||
			(b0 + (a0 * abs(C->row(0)[0]) + a1 * abs(C->row(1)[0]) + a2 * abs(C->row(2)[0])) < abs(B->col(0).dot(*D))) ||
			(b1 + (a0 * abs(C->row(0)[1]) + a1 * abs(C->row(1)[1]) + a2 * abs(C->row(2)[1])) < abs(B->col(1).dot(*D))) ||
			(b2 + (a0 * abs(C->row(0)[2]) + a1 * abs(C->row(1)[2]) + a2 * abs(C->row(2)[2])) < abs(B->col(2).dot(*D))) ||
			(((a1 * abs(C->row(2)[0]) + a2 * abs(C->row(1)[0])) + (b1 * abs(C->row(0)[2]) + b2 * abs(C->row(0)[1]))) < abs(C->row(1)[0] * A->col(2).dot(*D) - C->row(2)[0] * A->col(1).dot(*D))) ||
			(((a1 * abs(C->row(2)[1]) + a2 * abs(C->row(1)[1])) + (b0 * abs(C->row(0)[2]) + b2 * abs(C->row(0)[0]))) < abs(C->row(1)[1] * A->col(2).dot(*D) - C->row(2)[1] * A->col(1).dot(*D))) ||
			(((a1 * abs(C->row(2)[2]) + a2 * abs(C->row(1)[2])) + (b0 * abs(C->row(0)[1]) + b1 * abs(C->row(0)[0]))) < abs(C->row(1)[2] * A->col(2).dot(*D) - C->row(2)[2] * A->col(1).dot(*D))) ||
																																														   
			(((a0 * abs(C->row(2)[0]) + a2 * abs(C->row(0)[0])) + (b1 * abs(C->row(1)[2]) + b2 * abs(C->row(1)[1]))) < abs(C->row(2)[0] * A->col(0).dot(*D) - C->row(0)[0] * A->col(2).dot(*D))) ||
			(((a0 * abs(C->row(2)[1]) + a2 * abs(C->row(0)[1])) + (b0 * abs(C->row(1)[2]) + b2 * abs(C->row(1)[0]))) < abs(C->row(2)[1] * A->col(0).dot(*D) - C->row(0)[1] * A->col(2).dot(*D))) ||
			(((a0 * abs(C->row(2)[2]) + a2 * abs(C->row(0)[2])) + (b0 * abs(C->row(1)[1]) + b1 * abs(C->row(1)[0]))) < abs(C->row(2)[2] * A->col(0).dot(*D) - C->row(0)[2] * A->col(2).dot(*D))) ||
																																														   
			(((a0 * abs(C->row(1)[0]) + a1 * abs(C->row(0)[0])) + (b1 * abs(C->row(2)[2]) + b2 * abs(C->row(2)[1]))) < abs(C->row(0)[0] * A->col(1).dot(*D) - C->row(1)[0] * A->col(0).dot(*D))) ||
			(((a0 * abs(C->row(1)[1]) + a2 * abs(C->row(0)[1])) + (b0 * abs(C->row(2)[2]) + b2 * abs(C->row(2)[0]))) < abs(C->row(0)[1] * A->col(1).dot(*D) - C->row(1)[1] * A->col(0).dot(*D))) ||
			(((a0 * abs(C->row(1)[2]) + a2 * abs(C->row(0)[2])) + (b0 * abs(C->row(2)[1]) + b1 * abs(C->row(2)[0]))) < abs(C->row(0)[2] * A->col(1).dot(*D) - C->row(1)[2] * A->col(0).dot(*D)));
	}
	void Renderer::go_Bunny() {
		
		switch (scn->game_level)
		{
		case 1:
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			break;


		case 2:
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			break;
		case 3:
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			break;
		case 4: 
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			scn->data()._y -= 0.1;
			if (scn->data()._y <= -2) {
				scn->data()._y = -scn->data()._y;
				
			}
			break;
		case 5:	
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data().y_sign*exp(scn->data()._y), 0));
			
			if (scn->data().stage == 0) {
				scn->data()._y += 0.1;
				if (scn->data()._y >= -3.025) {
					scn->data().stage = 1;
				}
			}
			else if ( scn->data().stage==1) {
				scn->data()._y += 0.1;
				scn->data()._x += 1.5/32;
				if (scn->data()._y >= 0.105) {
					scn->data().stage = 2;
				}
			}
			else if (scn->data().stage == 2){
				
				scn->data()._y -= 0.1;
				scn->data()._x += 1.5 / 32;
				if (scn->data()._y <= -3) {
					scn->data().stage = 3;
					scn->data().y_sign = -1;
				}
			}
			else if(scn->data().stage == 3)
			{
				
				
				scn->data()._y += 0.1;
				scn->data()._x -= 1.5 / 32;
				if (scn->data()._y >= 0.105) {
					scn->data().stage = 4;
				}
			}
			else
			{
				scn->data()._x = -1.5;
				scn->data()._y -= 0.1;
			}
			scn->moveDown += 1;

			break;
		case 6:
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			if (scn->data().stage == 0) {
				scn->data()._y += 0.01;
				scn->data()._x += 0.01;
				if (scn->data()._y >= 0.2) {
					scn->data().stage = 1;
				}
			}
			else if (scn->data().stage == 1) {
				scn->data()._y -= 0.01;
				scn->data()._x += 0.01;
				if(scn->data()._x>=0.2)
					scn->data().stage = 2;
			}
			else if (scn->data().stage == 2) {
				scn->data()._y -= 0.01;
				scn->data()._x -= 0.01;
				if (scn->data()._y <= -0.2)
					scn->data().stage = 3;
			}
			else if (scn->data().stage == 3) {
				scn->data()._y += 0.01;
				scn->data()._x -= 0.01;
				if (scn->data()._x <= -0.2)
					scn->data().stage = 0;
			}
			break;
		case 7:
			scn->data().MyTranslate(Eigen::Vector3f(scn->data()._x, scn->data()._y, scn->data()._z));
			if (scn->data().stage == 0) {
				scn->data()._y += 0.01;
				scn->data()._x += 0.01;
				if (scn->data()._y >= 0.2) {
					scn->data().stage = 1;
				}
			}
			else if (scn->data().stage == 1) {
				scn->data()._y -= 0.01;
				scn->data()._x += 0.01;
				if (scn->data()._x >= 0.2)
					scn->data().stage = 2;
			}
			else if (scn->data().stage == 2) {
				scn->data()._y -= 0.01;
				scn->data()._x -= 0.01;
				if (scn->data()._y <= -0.2)
					scn->data().stage = 3;
			}
			else if (scn->data().stage == 3) {
				scn->data()._y += 0.01;
				scn->data()._x -= 0.01;
				if (scn->data()._x <= -0.2)
					scn->data().stage = 0;
			}
			break;
		default:
			break;
		}


	}
	void  Renderer::start_time() {
		time_start = time(NULL);
	}
	void Renderer::start_level_sound() {
		PlaySound(TEXT("soundEffects\\rattlesnake6.wav"), NULL, SND_FILENAME|SND_LOOP | SND_ASYNC);
	}
	
	void Renderer::snake_win_sound() {
		PlaySound(TEXT("soundEffects\\applause7.wav"), NULL, SND_FILENAME | SND_ASYNC);
	}
	void Renderer::snake_lose_sound() {
		PlaySound(TEXT("soundEffects\\boo3.wav"), NULL, SND_FILENAME | SND_ASYNC);
	}
	void Renderer::ReSetGame() {
		
		for (int i = scn->data_list.size() - 1; i >= 0; --i) {
			scn->erase_mesh(i);
		}
		
		scn->load_meshes_from_file("configuration.txt");
		scn->erase_mesh(0);
		scn->initEdges();
		locateCore1();
		line_less();
		
		snake_win = false;
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