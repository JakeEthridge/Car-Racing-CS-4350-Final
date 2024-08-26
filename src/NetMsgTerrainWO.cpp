#include "NetMsgTerrainWO.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"


using namespace Aftr;

NetMsgMacroDefinition(NetMsgSwitchTerrain);

NetMsgSwitchTerrain::NetMsgSwitchTerrain()
    : NetMsg(), useAnotherGrid(false), moveDownAmount(0.0f), rotateAmount(0.0f), moveNegativeXAmount(0.0f), movePositiveXAmount(0.0f){}

NetMsgSwitchTerrain::~NetMsgSwitchTerrain() {}

bool NetMsgSwitchTerrain::toStream(NetMessengerStreamBuffer& os) const {
    os << static_cast<int>(useAnotherGrid);
    os << moveDownAmount;
    os << rotateAmount;
    os << moveNegativeXAmount;
    os << movePositiveXAmount;
    return true;
}

bool NetMsgSwitchTerrain::fromStream(NetMessengerStreamBuffer& is) {
    int temp;
    is >> temp;
    useAnotherGrid = static_cast<bool>(temp);
    is >> moveDownAmount;
    is >> rotateAmount;
    is >> moveNegativeXAmount;
    is >> movePositiveXAmount;
    return true;
}

void NetMsgSwitchTerrain::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        // Debugging to check if the message arrives and is processed
        std::cout << "NetMsgSwitchTerrain arrived: useAnotherGrid = " << useAnotherGrid
            << ", moveDownAmount = " << moveDownAmount
            << ", rotateAmount = " << rotateAmount
            << ", moveNegativeXAmount = " << moveNegativeXAmount << std::endl;

        // Apply terrain changes
            glView->switchTerrain(useAnotherGrid);
        glView->moveTerrainDown(moveDownAmount);
        glView->rotateTerrain(rotateAmount);
        glView->moveTerrainNegativeX(moveNegativeXAmount);
        glView->moveTerrainPositiveX(movePositiveXAmount);
    }
}

