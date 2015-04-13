/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0
Source code name	: ch2_1_HelloPhysx
Reference Chapter	: Chapter-2: Basic Concepts

Description			: This example demonstrates the initialization, stepping and shutdown of PhysX SDK version 3.3.0
It is more like 'Hello World' program for PhysX SDK and contains minimal code. It mainly contains
three function which are as follows;

void InitPhysX();		//Initialize the PhysX SDK and create two actors.
void StepPhysX();		//Step PhysX simulation 300 times.
void ShutdownPhysX(); // Shutdown PhysX SDK

ConnectPVD();			//Function for the visualization of PhysX simulation (Optional)

This example code itself doesn't provide any implementation for rendering the PhysX objects.
However you can use PVD software to visualize the simulation. PVD can only be used in 'Debug' mode(configuration).
Please refer to last chapter (PhysX Visual Debugger) for more information.

=====================================================================
*/



#include <iostream> 
#include <vector>
#include <conio.h>
//random
#include <stdlib.h>
#include <time.h>
//
#include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API 
#include "PsString.h"
#include "windows/PsWindowsFile.h"

//-------Loading PhysX libraries (32bit only)----------//

#ifdef _DEBUG //If in 'Debug' load libraries for debug mode 
#pragma comment(lib, "PhysX3DEBUG_x86.lib")				//Always be needed  
#pragma comment(lib, "PhysX3CommonDEBUG_x86.lib")		//Always be needed
#pragma comment(lib, "PhysX3ExtensionsDEBUG.lib")		//PhysX extended library 
#pragma comment(lib, "PhysXVisualDebuggerSDKDEBUG.lib") //For PVD only 

#else //Else load libraries for 'Release' mode
#pragma comment(lib, "PhysX3_x86.lib")	
#pragma comment(lib, "PhysX3Common_x86.lib") 
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "PhysXVisualDebuggerSDK.lib")
#endif

using namespace std;
using namespace physx;

#define  FLUID_PARTICLE_NUMS 200
//--------------Global variables--------------//
static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK

PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f / 60.0f;		//Time-step value for PhysX simulation 
PxRigidDynamic					*gBox = NULL;				//Instance of box actor 
PxRigidDynamic					*gBall = NULL;				//Instance of ball actor
PxRigidStatic					*plane = NULL;				//Instance of a plane
PxCloth							*gCloth = NULL;				//Instance of Cloth
PxParticleFluid					*gFluid = NULL;				//Instance of Physx SPH Particle system

//Fluid Data Buffer
std::vector<PxU32> mTmpIndexArray;
std::vector<PxVec3> positions;
std::vector<PxVec3> positionsPrevious;
std::vector<PxVec3> velocities;
std::vector<PxVec3> forces;
std::vector<PxVec3> impulses;

PxU32* triangles;
PxVec3 *clothParticlePos;
//ContactInfos



//-----------PhysX function prototypes------------//
void InitPhysX();		//Initialize the PhysX SDK and create two actors. 
void StepPhysX();		//Step PhysX simulation 300 times.

void CreateTestSphere(PxVec3 pos, PxReal radius);//
void CreateCloth();		//
void CreateFluid();		//

void ShutdownPhysX();	//Shutdown PhysX SDK



void ConnectPVD();		//Function for the visualization of PhysX simulation (Optional and 'Debug' mode only) 


int DetectContactsBetweenParticleAndMesh(PxVec3 posParticle, PxVec3 velParticle, PxVec3 meshP0, PxVec3 meshP1, PxVec3 meshP2, float& u, float& v, float& w, PxVec3& n);


void main()
{

	InitPhysX();  //Initialize PhysX then create scene and actors
	ConnectPVD(); //Uncomment this function to visualize  the simulation in PVD

	//Simulate PhysX 300 times
	for (int i = 0; i <= 3000; i++)
	{
		//Step PhysX simulation
		if (gScene)
			StepPhysX();
		//Get current position of actor(box) and print it
		//PxVec3 boxPos = gBox->getGlobalPose().p;
		//cout << "Box current Position (" << boxPos.x << " " << boxPos.y << " " << boxPos.z << ")\n";
	}


	cout << "\nSimulation is done, shutting down PhysX!\n";

	//Shut down PhysX
	ShutdownPhysX();
	_getch();

}





void InitPhysX()
{
	//Creating foundation for PhysX
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);

	//Creating instance of PhysX SDK
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());

	if (gPhysicsSDK == NULL)
	{
		cerr << "Error creating PhysX3 device, Exiting..." << endl;
		exit(1);
	}


	//Creating scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());	//Descriptor class for scenes 

	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);		//Setting gravity
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);	//Creates default CPU dispatcher for the scene
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;	//Creates default collision filter shader for the scene
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;                //Enable CCD

	gScene = gPhysicsSDK->createScene(sceneDesc);				//Creates a scene 


	//Creating PhysX material
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5); //Creating a PhysX material



	//---------Creating actors-----------]

	//1-Creating static plane	 
	PxTransform planePos = PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));	//Position and orientation(transform) for plane actor  
	PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(planePos);								//Creating rigid static actor	
	plane->createShape(PxPlaneGeometry(), *material);												//Defining geometry for plane actor
	gScene->addActor(*plane);																		//Adding plane actor to PhysX scene


	//2-Creating dynamic cube																		 
	//PxTransform		boxPos(PxVec3(0.0f, 2.0f, 0.0f));												//Position and orientation(transform) for box actor 
	//PxBoxGeometry	boxGeometry(PxVec3(2, 2, 2));													//Defining geometry for box actor
	//gBox = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *material, 1.0f);						//Creating rigid static actor
	//gBox->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);

	//gScene->addActor(*gBox);																		//Adding box actor to PhysX scene


	//3-Creating static ball
	//PxTransform ballPos(PxVec3(5.0f, 2.0f, 0.0f));
	//PxSphereGeometry ballGeometry(PxReal(2.0f));
	//gBall = PxCreateDynamic(*gPhysicsSDK, ballPos, ballGeometry, *material, 1.0f);
	//gBall->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	//gScene->addActor(*gBall);
	
	//3.5-test


	//CreateTestSphere(PxVec3(2.0f, 20.0f, 0.0f), PxReal(0.7f));

	//4-Creating Fluid
	CreateFluid();

    //////////////////////////////////////////////////////////////////////////
    ///4.5 - For Paper
    //需要拿到的信息包括流体粒子的质量，流体粒子的位置、速度
    //int particleNum = gFluid->getMaxParticles();        //粒子个数
    //PxReal particleMass = gFluid->getParticleMass();    //粒子质量 暂时没有用

    // lock SDK buffers of *PxParticleSystem* ps for reading


    


    //粒子的位置速度和作用力需要在update的时候每次用Buffer读取
    //需要更新的信息主要是冲量
    //for (int i = 0; i < particleNum/2; i++)
    //{
    //    positions[i] = PxVec3(0, 10, 0);
    //}
    //PxStrideIterator<const PxVec3> posBuffer(&positions[0]);
    //PxStrideIterator<const PxU32> indexBuffer(&mTmpIndexArray[0]);
    //gFluid->setPositions(particleNum, indexBuffer, posBuffer);
    //OK!


    //////////////////////////////////////////////////////////////////////////

	//5-Creating Cloth
	CreateCloth();

    //PxMaterial* material = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5); //Creating a PhysX material
    PxRigidStatic *gParticle = NULL;
    //PxTransform gParticlePos(PxVec3(-1.5,-1.29,0));
    PxTransform gParticlePos(PxVec3(0, 10, 0));
    PxSphereGeometry gParticleGeometry(0.03f);
    PxShape* shape = gPhysicsSDK->createShape(gParticleGeometry, *material, false);

    gParticle = PxCreateStatic(*gPhysicsSDK, gParticlePos, *shape);

    gScene->addActor(*gParticle);


    PxRigidStatic *gParticle1 = NULL;
    //gParticle1 = PxCreateStatic(*gPhysicsSDK, PxTransform(PxVec3(-1.5, -0.99, -0.3)), *shape);
    gParticle1 = PxCreateStatic(*gPhysicsSDK, PxTransform(PxVec3(0, 10, 10)), *shape);
    gScene->addActor(*gParticle1);

    PxRigidStatic *gParticle2 = NULL;
    //gParticle2 = PxCreateStatic(*gPhysicsSDK, PxTransform(PxVec3(-1.19, -0.99, -0.3)), *shape);
    gParticle2 = PxCreateStatic(*gPhysicsSDK, PxTransform(PxVec3(10, 10, 0)), *shape);
    gScene->addActor(*gParticle2);
	//6-Contact Modification
	
	
}

void CreateTestSphere(PxVec3 pos, PxReal radius)
{
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5); //Creating a PhysX material
	PxRigidDynamic *gParticle = NULL;
	PxTransform gParticlePos(pos);
	PxSphereGeometry gParticleGeometry(radius);
	gParticle = PxCreateDynamic(*gPhysicsSDK, gParticlePos, gParticleGeometry, *material, 1.0f);
	gParticle->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*gParticle);
}
void CreateCloth()
{

	//PxTransform gPose = PxTransform(PxVec3(-3, 3, 8));
    PxTransform gPose = PxTransform(PxVec3(0, 0, 0));
	// create regular mesh
	PxU32 resolution = 50;
	PxU32 numParticles = resolution*resolution;
	PxU32 numTriangles = 2 * (resolution - 1)*(resolution - 1);

	// create cloth particles
	PxClothParticle* particles = new PxClothParticle[numParticles];
	PxVec3 center(1.5f, 1.3f, 0.0f);
	PxVec3 delta = 1.0f / (resolution - 1) * PxVec3(15.0f, 15.0f, 15.0f);
	PxClothParticle* pIt = particles;
	for (PxU32 i = 0; i < resolution; ++i)
	{
		for (PxU32 j = 0; j < resolution; ++j, ++pIt)
		{
			// if weight == 0, particle is fixed
			pIt->invWeight = j + 1 < resolution ? 0.0f : 0.5f;
			pIt->pos = delta.multiply(PxVec3(PxReal(i),
				PxReal(j), -PxReal(j))) - center;
		}
	}
	// 现在所有布料粒子的位置已经确定 在每个的pos里面
	// create triangles
	triangles = new PxU32[3 * numTriangles];
	PxU32* iIt = triangles;
	for (PxU32 i = 0; i < resolution - 1; ++i)
	{
		for (PxU32 j = 0; j < resolution - 1; ++j)
		{
			PxU32 odd = j & 1u, even = 1 - odd;
			*iIt++ = i*resolution + (j + odd);
			*iIt++ = (i + odd)*resolution + (j + 1);
			*iIt++ = (i + 1)*resolution + (j + even);
			*iIt++ = (i + 1)*resolution + (j + even);
			*iIt++ = (i + even)*resolution + j;
			*iIt++ = i*resolution + (j + odd);
		}
	}
	// 现在所有对应三角形的位置已经确定 在triangle里面
	// create fabric from mesh
	PxClothMeshDesc meshDesc;
	meshDesc.points.count = numParticles;
	meshDesc.points.stride = sizeof(PxClothParticle);
	meshDesc.points.data = particles;

	meshDesc.invMasses.count = numParticles;
	meshDesc.invMasses.stride = sizeof(PxClothParticle);
	meshDesc.invMasses.data = &particles->invWeight;

	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);
	meshDesc.triangles.data = triangles;


	// cook fabric
	PxClothFabric* fabric = PxClothFabricCreate(*gPhysicsSDK, meshDesc, PxVec3(0, 1, 0));

	//delete[] triangles;

	// create cloth
	gCloth = gPhysicsSDK->createCloth(gPose, *fabric, particles, PxClothFlags());

	fabric->release();
	delete[] particles;

	// 240 iterations per/second (4 per-60hz frame)
	gCloth->setSolverFrequency(240.0f);

	// set self collision 
	gCloth->setSelfCollisionDistance(0.1f);
	gCloth->setSelfCollisionStiffness(1.0f);

	// add collision shapes
	gCloth->setClothFlag(PxClothFlag::eSCENE_COLLISION,true);

	// set friction
	gCloth->setFrictionCoefficient(0.8f);

	// Two spheres located on the x-axis

	PxClothCollisionSphere spheres[2] =
	{
		PxClothCollisionSphere(PxVec3(-1.0f, 0.0f, 0.0f), 2.5f),
		PxClothCollisionSphere(PxVec3(1.0f, 0.0f, 0.0f), 2.25f)
	};
	//gCloth->setCollisionSpheres(spheres, 2);
	//gCloth->addCollisionCapsule(0, 1);



	gScene->addActor(*gCloth);


}


void CreateFluid()
{
	// create particle system in PhysX SDK
	gFluid = gPhysicsSDK->createParticleFluid(FLUID_PARTICLE_NUMS);
	gFluid->setGridSize(5.0f);
	gFluid->setMaxMotionDistance(0.01f);
	gFluid->setRestOffset(0.8f*0.3f);
	gFluid->setContactOffset(0.8f*0.3f * 2);
	gFluid->setDamping(0.0f);
	gFluid->setRestitution(0.3f);
	gFluid->setDynamicFriction(0.001f);
	gFluid->setRestParticleDistance(0.8f);
	gFluid->setViscosity(60.0f);
	gFluid->setStiffness(45.0f);
	gFluid->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);
	//particle mass for floating box
	gFluid->setParticleMass(100.0f);
	// CPU now
	// add particle system to scene, in case creation was successful
	gScene->addActor(*gFluid);

	static const PxFilterData collisionGroupWaterfall(0, 0, 1, 0);
	gFluid->setSimulationFilterData(collisionGroupWaterfall);

	srand(time(NULL));
	PxVec3 globalPosition(1.0f, 10.0f, -10.0f);
    for (int i = 0; i < FLUID_PARTICLE_NUMS; ++i)
	{
		mTmpIndexArray.push_back(i);
		float num1, num2, num3;
		num1 = (rand() % 100)*0.01;
		num2 = (rand() % 100)*0.01;
		num3 = (rand() % 100)*0.01;
        //num2 = i / 3.f;
        //num1 = 0.f;
        //num3 = 0.f;
		PxVec3 pos(num1, num2, num3);
		pos = pos + globalPosition;
		//PxVec3 vel(-num1, -num2, -num3);
		PxVec3 vel(0, 0, 0);
		positions.push_back(pos);
        //初始化前一刻的位置
        positionsPrevious.push_back(pos);
		velocities.push_back(vel);
        //这里初始化力冲量和速度为0
        forces.push_back(vel);
        impulses.push_back(vel);
		//cout << num1 << num2 << num3 << endl;
	}

	PxParticleCreationData particleCreationData;

    particleCreationData.numParticles = FLUID_PARTICLE_NUMS;
	particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(&mTmpIndexArray[0]);
	particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(&positions[0]);
	particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(&velocities[0]);
	// set fluid collisions
	


	// last step - Create
	gFluid->createParticles(particleCreationData);

}


void StepPhysX()					//Stepping PhysX
{
	gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
	gScene->fetchResults(true);		//Block until the simulation run is completed


    //这里加入自己定义的碰撞检测的内容
    //首先拿到各个粒子的参数
    //int particleNum = gFluid->getMaxParticles();        //粒子个数
    //PxReal particleMass = gFluid->getParticleMass();    //粒子质量

    //接下来获取粒子运动时的速度和坐标
    PxParticleReadData* rd = gFluid->lockParticleReadData();
    
    // access particle data from PxParticleReadData
    if (rd)
    {
        PxStrideIterator<const PxParticleFlags> flagsIt(rd->flagsBuffer);
        PxStrideIterator<const PxVec3> positionIt(rd->positionBuffer);
        PxStrideIterator<const PxVec3> velocityIt(rd->velocityBuffer);
        for (unsigned i = 0; i < rd->validParticleRange; ++i, ++flagsIt, ++positionIt, ++velocityIt)
        {
            if (*flagsIt & PxParticleFlag::eVALID)
            {
                // access particle position
                const PxVec3& position = *positionIt;
                const PxVec3& velocity = *velocityIt;
                //更新此刻流体粒子的位置
                positions[i] = position;
                //cout << "pos:" << position.x << " " << position.y << " " << position.z << " " << endl;
                //cout << "vel:" << velocity.x << " " << velocity.y << " " << velocity.z << " " << endl;

                //接下来要根据已经获取的布料所有三角面片的坐标进行碰撞检测
                //test case 1
                //PxVec3 x1(0.f, 10.f, 0.f);
                //PxVec3 x2(0.f, 10.f, 10.f);
                //PxVec3 x3(10.f, 10.f, 0.f);
                //这里获得布料的所有粒子位置
                PxClothParticleData* data = gCloth->lockParticleData(PxDataAccessFlag::eREADABLE);
                PxU32 clothParticleNum = gCloth->getNbParticles();
                clothParticlePos = new PxVec3[clothParticleNum];
                for (PxU32 j = 0; j < clothParticleNum; ++j)
                {
                    clothParticlePos[j] = data->particles[j].pos;            
                }
                //重组所有的三角面片
                bool collisionDetected = false;
                data->unlock();
                for (PxU32 j = 0; j < 14406; j=j+3) //这里的数字是所有点的总数
                {

                    float u, v, w = 0.0f;
                    PxVec3 n(0.f, 0.f, 0.f);
                    
                    PxVec3 &x3 = clothParticlePos[triangles[j]];
                    PxVec3 &x2 = clothParticlePos[triangles[j + 1]];
                    PxVec3 &x1 = clothParticlePos[triangles[j + 2]];
                    //在150413的版本中，对于快速移动的布料和流体 无法进行正确的碰撞检测 隧穿是由于布料的速度过快导致的
                    if (DetectContactsBetweenParticleAndMesh(position, velocity, x1, x2, x3, u, v, w, n))
                    {
                        //cout << "这里碰撞了" << endl;

                        //碰撞发生之后 需要往布料和流体粒子上作用一个冲量
                        //根据Huber-2013 对于冲量的计算 可以加入一个布料的厚度来改进公式
                        n.normalize();
                        float d = 0.12 - (position - u*x1 - v*x2 - w*x3).dot(n);
                        // 0.02这里表示一个临时的布料厚度
                        // 接下来 I = -kd * Dt
                        int particleNum = gFluid->getMaxParticles();        //粒子个数
                        PxReal particleMass = gFluid->getParticleMass();    //粒子质量 暂时没有用
                        //粒子的位置速度和作用力需要在update的时候每次用Buffer读取
                        //需要更新的信息主要是冲量
                        //impulses[i] = gCloth->getStretchConfig(PxClothFabricPhaseType::eBENDING).stiffness*d*gTimeStep*(-1)*n;                        
                        impulses[i] = 10*gFluid->getParticleMass() * (velocity.magnitude()) * (n);
                        //强制更新到前一刻的位置
                        //强制考虑布料厚度问题
                        positionsPrevious[i] = positionsPrevious[i] + (0.1f)*(n);
                        //positionsPrevious[i].y += 0.1f;
                        positions[i] = positionsPrevious[i];

                        collisionDetected = true;
                    }
                    if (collisionDetected)
                    {
                        PxStrideIterator<const PxVec3> impluseBuffer(&impulses[0]);
                        PxStrideIterator<const PxVec3> positionBuffer(&positions[0]);
                        PxStrideIterator<const PxU32> indexBuffer(&mTmpIndexArray[0]);
                        gFluid->addForces(gFluid->getMaxParticles(), indexBuffer, impluseBuffer, PxForceMode::eIMPULSE);
                        rd->unlock();
                        gFluid->setPositions(gFluid->getMaxParticles(), indexBuffer, positionBuffer);
                        
                        //OK!

                        //布料这一块也需要更新冲量

                        //NOK

                        //更新一些临时变量
                        collisionDetected = false;
                        impulses[i] = PxVec3(PxZero);
                    }

                }
                //释放内存
                delete clothParticlePos;

                //保存上一次计算后流体粒子的位置和速度
                positionsPrevious[i] = position;

            }

        }

        // return ownership of the buffers back to the SDK
        rd->unlock();
    }
    //其次拿到规整的布料粒子的位置
}


void ShutdownPhysX()				//Shutdown PhysX
{
	gPhysicsSDK->release();			//Removes any actors,  particle systems, and constraint shaders from this scene
	gFoundation->release();			//Destroys the instance of foundation SDK
}



void ConnectPVD()					//Function for the visualization of PhysX simulation (Optional and 'Debug' mode only) 
{
	// check if PvdConnection manager is available on this platform
	if (gPhysicsSDK->getPvdConnectionManager() == NULL)
		return;

	// setup connection parameters
	const char*     pvd_host_ip = "127.0.0.1";  // IP of the PC which is running PVD
	int             port = 5425;         // TCP port to connect to, where PVD is listening
	unsigned int    timeout = 100;          // timeout in milliseconds to wait for PVD to respond,
	// consoles and remote PCs need a higher timeout.
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

	// and now try to connect
	debugger::comm::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(),
		pvd_host_ip, port, timeout, connectionFlags);

}



//Help functions

static void FixSeparators(char* path)
{
	for (unsigned i = 0; i < strlen(path); ++i)
		if (path[i] == '\\' || path[i] == '/')
			path[i] = '\\';
}


const char* findPath(const char* path)
{
	static std::vector<char*>* gSearchPaths = NULL;

	if (!gSearchPaths)
		return NULL;

	static char fullPath[512];
	FILE* file = NULL;
	const PxU32 numSearchPaths = (PxU32)gSearchPaths->size();
	for (PxU32 i = 0; i < numSearchPaths; i++)
	{
		const char* prefix = (*gSearchPaths)[i];
		physx::string::strcpy_s(fullPath, 512, prefix);
		physx::string::strcat_s(fullPath, 512, path);
		FixSeparators(fullPath);
		physx::shdfnd::fopen_s(&file, fullPath, "rb");
		if (file) break;
	}
	if (file)
	{
		fclose(file);
		return fullPath;
	}
	else
		return path;
}


//实时碰撞检测算法技术P132
int DetectContactsBetweenParticleAndMesh(PxVec3 posParticle, PxVec3 velParticle, PxVec3 meshP0, PxVec3 meshP1, PxVec3 meshP2, float& u, float& v, float& w, PxVec3& n)
{
    //if (posParticle.y<=10.1)
    //{
    //    cout << endl;
    //}
    PxVec3 &a = meshP0;
    PxVec3 &b = meshP1;
    PxVec3 &c = meshP2;

    PxVec3 &p = posParticle;
    PxVec3 &q = posParticle + gTimeStep*velParticle*1.2;

    PxVec3 ab = b - a;
    PxVec3 ac = c - a;
    PxVec3 qp = p - q;

    n = ab.cross(ac);

    float d = qp.dot(n);

    if (d <= 0.0f)
    {
        return 0;
    }


    PxVec3 ap = p - a;
    float t = ap.dot(n);
    if (t < 0.0f)
    {
        return 0;
    }
    if (t > d)
    {
        return 0;
    }
    

    PxVec3 e = qp.cross(ap);
    v = ac.dot(e);
    if (v<0.0f || v>d)
    {
        return 0;
    }
    w = -ab.dot(e);
    if (w < 0.0f || d < (v + w))
    {
        return 0;
    }

    float odd = 1.0f / d;
    t *= odd;
    v *= odd;
    w *= odd;
    u = 1.0f - v - w;

    return 1;
}