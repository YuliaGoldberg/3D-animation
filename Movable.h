#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <iostream>

class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4f MakeTransScale(Eigen::Vector3f amt);
	Eigen::Matrix4f MakeTransIn();
	
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt, Movable* scene);
	void MyRotate(Eigen::Vector3f rotAxis, float angle);
	Eigen::Matrix3f GetRotation();
	void MyScale(Eigen::Vector3f amt);
	void SetCenterOfRotation(Eigen::Vector3d amt);
	void PrintRotation();
	void RotInSys(Eigen::Vector3f rotAxis, float angle);
	//Eigen::Vector3d GetCenterOfRotation();

private:

	Eigen::Transform<float,3,Eigen::Affine> Tout;
	Eigen::Transform<float, 3, Eigen::Affine> Tin;
};

