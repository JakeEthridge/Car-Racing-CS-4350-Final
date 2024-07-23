#include "NetMsgTerrainWO.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"

using namespace Aftr;

NetMsgMacroDefinition(NetMsgSwitchTerrain);

NetMsgSwitchTerrain::NetMsgSwitchTerrain() : NetMsg(), useAnotherGrid(false) {}

NetMsgSwitchTerrain::~NetMsgSwitchTerrain() {}

bool NetMsgSwitchTerrain::toStream(NetMessengerStreamBuffer& os) const {
    os << static_cast<int>(useAnotherGrid);
    return true;
}

bool NetMsgSwitchTerrain::fromStream(NetMessengerStreamBuffer& is) {
    int temp;
    is >> temp;
    useAnotherGrid = static_cast<bool>(temp);
    return true;
}

void NetMsgSwitchTerrain::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        // Debugging to check if the message arrives and is processed
        std::cout << "NetMsgSwitchTerrain arrived: useAnotherGrid = " << useAnotherGrid << std::endl;

        // Ensure the terrain is only switched once
        static bool terrainSwitched = false;
        if (!terrainSwitched) {
            glView->switchTerrain(useAnotherGrid);
            terrainSwitched = true;
        }
    }
}
