#pragma once
#include "NetMsg_callback_decl.h"
#include "ManagerSerializableNetMsgMap.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include <memory>
#include "NetMsg.h"
#include <vector>
#include <memory>
#include <string>
#include "NetMsgCreateRawWO.h"
#include "GLViewSpeedRacer.h"
#include "GLView.h"
#include <new.h>
#include <ManagerGLView.h>
#include <WO.h>

using namespace Aftr;

NetMsgMacroDefinition(NetMsgCreateRawWO);

NetMsgCreateRawWO::NetMsgCreateRawWO()
{
	this->xPos;
	this->yPos;
	this->zPos;
}

NetMsgCreateRawWO::~NetMsgCreateRawWO()
{

}


bool NetMsgCreateRawWO::toStream(NetMessengerStreamBuffer& os) const
{
	os << this->xPos;
	os << this->yPos;
	os << this->zPos;

	os << this->x_current;
	os << this->y_current;
	os << this->z_current;

	os << this->last_x;
	os << this->last_y;
	os << this->last_z;

	return true;

}
bool NetMsgCreateRawWO::fromStream(NetMessengerStreamBuffer& is)
{
	is >> this->xPos;
	is >> this->yPos;
	is >> this->zPos;

	is << this->x_current;
	is << this->y_current;
	is << this->z_current;

	is << this->last_x;
	is << this->last_y;
	is << this->last_z;
	//onMessageArrived();
	return true;
}


void NetMsgCreateRawWO::onMessageArrived()
{
	Vector currentPosition = ManagerGLView::getGLViewT<GLViewSpeedRacer>()->car1->getPosition();
	ManagerGLView::getGLViewT<GLViewSpeedRacer>()->car1;
	ManagerGLView::getGLViewT<GLViewSpeedRacer>()->car1->setPosition(xPos, yPos, zPos);
	ManagerGLView::getGLViewT<GLViewSpeedRacer>()->car1->rotateAboutGlobalZ(rotationAngle);
	//ManagerGLView::getGLViewT<GLViewAssignment5>()->red_cube->rotateAboutGlobalZ(rotationZ);
	//ManagerGLView::getGLViewT<GLViewAssignment5>()->red_cube->rotateAboutGlobalX(rotationX);
	//ManagerGLView::getGLViewT<GLViewAssignment5>()->red_cube->setPosition(x_current - last_x, y_current - last_y, z_current - last_z);
	std::cout << this->toString() << std::endl;

	
}





std::string NetMsgCreateRawWO::toString() const
{
	std::stringstream ss;
	ss << NetMsg::toString();
	ss << "      Payload" << this->xPos << this->yPos << this->zPos << "...\n";
	ss << "TOTOTOTOOTOTO" << std::endl;
	return ss.str();

}




