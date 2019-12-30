#include "Movable.h"

Movable::Movable()
{
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	 
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return (Tout*Tin).matrix();
}
Eigen::Matrix4f Movable::MakeTransScale(Eigen::Vector3f amt)
{
	Eigen::Transform<float, 3, Eigen::Affine> temp_tout = Tout;
	temp_tout.translate(amt);
	return (temp_tout) * Tin.matrix();
}
Eigen::Matrix4f Movable::MakeTransIn()
{
	return Tin.matrix();
}

//Eigen::Matrix4f Movable::MakeTransScale()
//{
//	return Tout.matrix()*Tin.matrix();
//}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
}
void Movable::MyTranslate(Eigen::Vector3f amt, Movable* scene)
{
	Eigen::Matrix3f mat = scene->Tout.rotation().matrix().inverse();
	MyTranslate(mat*amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
}
Eigen::Matrix3f Movable::GetRotation() {
	return Tout.rotation();
}
void Movable::MyScale(Eigen::Vector3f amt)
{
	Tout.scale(amt);
	
}

void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
{
	Tin.translate(-(amt).cast<float>());
	Tout.translate(amt.cast<float>()); // Tout
	
}
void Movable::PrintRotation() {
	std::cout << Tout.rotation() << std::endl;
}

void Movable::RotInSys(Eigen::Vector3f rotAxis, float angle)
{
	Eigen::Matrix3f inverse_tout = Tout.rotation().inverse();
	Tout.rotate(Eigen::AngleAxisf(angle, inverse_tout*rotAxis));
}
//void Movable::MyTranslate(Eigen::Vector3f amt, bool preRotation)
//{
//	if (preRotation)
//		T.pretranslate(amt); // Tout
//	else
//		T.translate(amt); // Tout
//}
////angle in radians
//void Movable::MyRotate(Eigen::Matrix3d &rot, float angle)
//{
//	T.rotate(Eigen::AngleAxisf(angle, rot.normalized()));
//	// Tout.rotate(...);
//}
//
//void Movable::MyScale(Eigen::Vector3f amt)
//{
//	T.scale(amt);
//	// Tin.scale(amt);
//}
//
//Eigen::Matrix4d Movable::MakeTransd()
//{
//	return T.matrix().cast<double>;
//}
//
//void Movable::TranslateInSystem(Eigen::Matrix4d mat, Eigen::Vector3d amt, bool preRotation)
//{
//	MyTranslate(Mat.block<3, 3>(0, 0).transpose() * amt, preRotation);
//}
//

//
//void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tin.translate(-amt);
//	T.translate(amt); // Tout
//}
//
//Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return -Tin.translation();
//}