#include "Terrain.h"

Aftr::WOGridECEFElevationPhysX::WOGridECEFElevationPhysX() :
    Aftr::IFace(this)
{
}

Aftr::WOGridECEFElevationPhysX::~WOGridECEFElevationPhysX()
{
    this->Aftr::WO::~WO();
}

Aftr::WOGridECEFElevationPhysX* Aftr::WOGridECEFElevationPhysX::New(physx::PxPhysics* pxPhysics, physx::PxScene* pxScene,
    Aftr::VectorD upperLeft, Aftr::VectorD lowerRight, Aftr::VectorD offset, Aftr::VectorD scale, const std::string& elevationPath, const std::string& texturePath)
{
    WOGridECEFElevationPhysX* grid = new WOGridECEFElevationPhysX();
    grid->onCreate(pxPhysics, pxScene, upperLeft, lowerRight, offset, scale, elevationPath, texturePath);
    return grid;
}


void Aftr::WOGridECEFElevationPhysX::onCreate(physx::PxPhysics* pxPhysics, physx::PxScene* pxScene,
    Aftr::VectorD upperLeft, Aftr::VectorD lowerRight, Aftr::VectorD offset, Aftr::VectorD scale, const std::string& elevationPath, const std::string& texturePath)
{
    this->WOGridECEFElevation::onCreate(upperLeft, lowerRight, 0, offset, scale, elevationPath, 0, true, 0);
    this->upon_async_model_loaded([this, offset, pxPhysics, pxScene, texturePath] {

        for (size_t i = 0; i < this->getModel()->getModelDataShared()->getModelMeshes().size(); i++)
        {
            this->getModel()->getModelDataShared()->getModelMeshes().at(i)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(texturePath);
            this->getModel()->isUsingBlending(false);

        }

        size_t vertexListSize = this->getModel()->getModelDataShared()->getCompositeVertexList().size();
        size_t indexListSize = this->getModel()->getModelDataShared()->getCompositeIndexList().size();

        this->vertexListCopy = new float[vertexListSize * 3];
        this->indiciesCopy = new unsigned int[indexListSize];


        for (size_t i = 0; i < vertexListSize; i++) {
            this->vertexListCopy[i * 3 + 0] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).x;
            this->vertexListCopy[i * 3 + 1] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).y;
            this->vertexListCopy[i * 3 + 2] = this->getModel()->getModelDataShared()->getCompositeVertexList().at(i).z;
        }
        for (size_t i = 0; i < indexListSize; i++) {
            indiciesCopy[i] = this->getModel()->getModelDataShared()->getCompositeIndexList().at(i);
        }

        physx::PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = vertexListSize;
        meshDesc.points.stride = sizeof(float) * 3;
        meshDesc.points.data = this->vertexListCopy;

        meshDesc.triangles.count = indexListSize / 3;
        meshDesc.triangles.stride = 3 * sizeof(unsigned int);
        meshDesc.triangles.data = this->indiciesCopy;

        physx::PxDefaultMemoryOutputStream writeBuffer;
        physx::PxTriangleMeshCookingResult::Enum result;
        physx::PxCookingParams cookingParams(pxPhysics->getTolerancesScale());
        bool status = PxCookTriangleMesh(cookingParams, meshDesc, writeBuffer, &result);
        if (!status) {
            std::cout << "Failed to create triangular mesh" << std::endl;
            std::cin.get();
        }

        physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        physx::PxTriangleMesh* mesh = pxPhysics->createTriangleMesh(readBuffer);

        physx::PxMaterial* gMaterial = pxPhysics->createMaterial(0.5f, 0.5f, 0.6f);
        physx::PxShape* shape = pxPhysics->createShape(physx::PxTriangleMeshGeometry(mesh), *gMaterial, true);

        auto p1 = this->getModel()->getPose();
        auto pose = this->getModel()->getPose();

        int val = 0;
        float disMat[16];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                disMat[val] = pose.at(i, j);
                val++;
            }
        }
        auto t = physx::PxTransform(physx::PxMat44(disMat));

        auto rigid = pxPhysics->createRigidStatic(t);
        bool b = rigid->attachShape(*shape);

        this->actor = rigid;
        this->actor->userData = this;
        pxScene->addActor(*actor);

        });
}
