/*
=====================================================================
Author				: Shin Shi
Compiler used		: Visual Studio 2013 Ultimate Update 4
PhysX SDK version	: 3.3.2
Source code name	: ph3_fluid


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


//-------Loading PhysX libraries (32bit only)----------//

#ifdef _DEBUG //If in 'Debug' load libraries for debug mode 
#pragma comment(lib, "PhysX3DEBUG_x86.lib")				//Always be needed  
#pragma comment(lib, "PhysX3CommonDEBUG_x86.lib")		//Always be needed
#pragma comment(lib, "PhysX3ExtensionsDEBUG.lib")		//PhysX extended library 
#pragma comment(lib, "PhysXVisualDebuggerSDKDEBUG.lib") //For PVD only 
#pragma comment(lib, "PhysX3GpuDEBUG_x86.lib")			//For GPU CUDA

#else //Else load libraries for 'Release' mode
#pragma comment(lib, "PhysX3_x86.lib")	
#pragma comment(lib, "PhysX3Common_x86.lib") 
#pragma comment(lib, "PhysX3Extensions.lib")
#pragma comment(lib, "PhysXVisualDebuggerSDK.lib")
#pragma comment(lib, "PhysX3Gpu_x86.lib")	
#endif


using namespace std;
using namespace physx;


//--------------Global variables--------------//
static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK

PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f / 60.0f;		//Time-step value for PhysX simulation 
PxRigidDynamic					*gBox = NULL;				//Instance of box actor 
PxCloth*						gCloth = NULL;				//Instance of Cloth
PxParticleFluid*				gFluid = NULL;				//Instance of Physx SPH Particle system
//Fluid Data Buffer
std::vector<PxU32> mTmpIndexArray;
std::vector<PxVec3> positions;
std::vector<PxVec3> velocities;

//-----------PhysX function prototypes------------//
void CreateCloth(PxVec3 globalPosition);		//Create cloth
void CreateFluid();
						//Create Physx SPH Particles

void InitPhysX();		//Initialize the PhysX SDK and create two actors. 
void StepPhysX();		//Step PhysX simulation 300 times.
void ShutdownPhysX();	//Shutdown PhysX SDK

void ConnectPVD();		//Function for the visualization of PhysX simulation (Optional and 'Debug' mode only) 





void main()
{

	InitPhysX();  //Initialize PhysX then create scene and actors
	ConnectPVD(); //Uncomment this function to visualize  the simulation in PVD

	//Simulate PhysX 300 times
	for (int i = 0; i <= 8000; i++)
	{
		//Step PhysX simulation
		if (gScene)
			StepPhysX();

		//Get current position of actor(box) and print it
		//PxVec3 boxPos = gBox->getGlobalPose().p;
		//cout << "Box current Position (" << boxPos.x << " " << boxPos.y << " " << boxPos.z << ")\n";


		if (i == 1300)
		{

			//Creating PhysX material
			PxMaterial* material = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5); //Creating a PhysX material
			PxRigidDynamic					*gBox1 = NULL;
			PxTransform		boxPos(PxVec3(0.0f, 5.0f, 0.0f));												//Position and orientation(transform) for box actor 
			PxBoxGeometry	boxGeometry(PxVec3(0.4, 0.4, 0.4));											//Defining geometry for box actor
			gBox1 = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *material, 1.0f);		//Creating rigid static actor
			gBox1->setMass(0.01f);
			gScene->addActor(*gBox1);

		}

	}

	cout << "\nSimulation is done, shutting down PhysX!\n";

	//Shut down PhysX
	ShutdownPhysX();
	_getch();

}

void CreateCloth(PxVec3 globalPosition)
{

	PxTransform gPose = PxTransform(PxVec3(0, 0, 0));

	// create regular mesh
	PxU32 resolution = 200;
	PxU32 numParticles = resolution*resolution;
	PxU32 numTriangles = 2 * (resolution - 1)*(resolution - 1);

	// create cloth particles
	PxClothParticle* particles = new PxClothParticle[numParticles];
	PxVec3 center(0.5f, 0.3f, 0.0f);
	PxVec3 delta = 1.0f / (resolution - 1) * PxVec3(15.0f, 15.0f, 15.0f);
	PxClothParticle* pIt = particles;
	for (PxU32 i = 0; i < resolution; ++i)
	{
		for (PxU32 j = 0; j < resolution; ++j, ++pIt)
		{
			pIt->invWeight = j + 1 < resolution ? 1.0f : 0.0f;
			pIt->pos = delta.multiply(PxVec3(PxReal(i),
				PxReal(j), -PxReal(j))) - center;
		}
	}

	// create triangles
	PxU32* triangles = new PxU32[3 * numTriangles];
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

	delete[] triangles;

	// create cloth
	gCloth = gPhysicsSDK->createCloth(gPose, *fabric, particles, PxClothFlags(0));

	fabric->release();
	delete[] particles;

	// 240 iterations per/second (4 per-60hz frame)
	gCloth->setSolverFrequency(240.0f);


	// set global pose
	gCloth->setGlobalPose(PxTransform(globalPosition));
	//

	// set self collision 
	gCloth->setSelfCollisionDistance(0.1f);
	gCloth->setSelfCollisionStiffness(1.0f);


	gScene->addActor(*gCloth);


}


void CreateFluid()
{
	// create particle system in PhysX SDK
	gFluid = gPhysicsSDK->createParticleFluid(800);
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
	for (int i = 0; i < 800; ++i)
	{
		mTmpIndexArray.push_back(i);
		float num1, num2, num3;
		num1 = (rand() % 100)*0.01;
		num2 = (rand() % 100)*0.01;
		num3 = (rand() % 100)*0.01;
		PxVec3 pos(num1, num2, num3);
		//PxVec3 vel(-num1, -num2, -num3);
		PxVec3 vel(0, 0, 0);
		positions.push_back(pos);
		velocities.push_back(vel);
		//cout << num1 << num2 << num3 << endl;
	}

	PxParticleCreationData particleCreationData;

	particleCreationData.numParticles = 800;
	particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(&mTmpIndexArray[0]);
	particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(&positions[0]);
	particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(&velocities[0]);

	gFluid->createParticles(particleCreationData);

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

	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);				//Setting gravity
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);	//Creates default CPU dispatcher for the scene
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;	//Creates default collision filter shader for the scene

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
	PxTransform		boxPos(PxVec3(2.0f, 0.0f, 0.0f));							//Position and orientation(transform) for box actor 
	PxBoxGeometry	boxGeometry(PxVec3(2, 2, 2));								//Defining geometry for box actor
	gBox = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *material, 1.0f);	//Creating rigid static actor
	gScene->addActor(*gBox);													//Adding box actor to PhysX scene

	PxRigidStatic *gBowl1 = NULL;
	PxTransform		bowlPos1(PxVec3(-2.0f, 2.0f, 0.0f));						//Position and orientation(transform) for box actor 
	PxBoxGeometry	bowlGeometry(PxVec3(0.1, 2, 2));							//Defining geometry for box actor
	gBowl1 = PxCreateStatic(*gPhysicsSDK, bowlPos1, bowlGeometry, *material);	//Creating rigid static actor
	gScene->addActor(*gBowl1);													//Adding box actor to PhysX scene
	
	PxRigidStatic *gBowl2 = NULL;
	PxTransform		bowlPos2(PxVec3(2.0f, 2.0f, 0.0f));							//Position and orientation(transform) for box actor 
	PxBoxGeometry	bowlGeometry2(PxVec3(0.1, 2, 2));							//Defining geometry for box actor
	gBowl2 = PxCreateStatic(*gPhysicsSDK, bowlPos2, bowlGeometry2, *material);	//Creating rigid static actor
	gScene->addActor(*gBowl2);

	PxRigidStatic *gBowl3 = NULL;
	PxTransform		bowlPos3(PxVec3(0.0f, 2.0f, 2.0f));							//Position and orientation(transform) for box actor 
	PxBoxGeometry	bowlGeometry3(PxVec3(2, 2, 0.1));							//Defining geometry for box actor
	gBowl3 = PxCreateStatic(*gPhysicsSDK, bowlPos3, bowlGeometry3, *material);	//Creating rigid static actor
	gScene->addActor(*gBowl3);

	PxRigidStatic *gBowl4 = NULL;
	PxTransform		bowlPos4(PxVec3(0.0f, 2.0f, -2.0f));						//Position and orientation(transform) for box actor 
	PxBoxGeometry	bowlGeometry4(PxVec3(2, 2, 0.1));							//Defining geometry for box actor
	gBowl4 = PxCreateStatic(*gPhysicsSDK, bowlPos4, bowlGeometry4, *material);	//Creating rigid static actor
	gScene->addActor(*gBowl4);

	//3-Creating dynamic cloth
	{
//		CreateCloth(PxVec3(0, 10, 10));
//		CreateCloth(PxVec3(0, 0, 10));
//		gScene->setClothInterCollisionDistance(0.5f);
//		gScene->setClothInterCollisionStiffness(1.0f);
//
//		gCloth->addCollisionPlane(PxClothCollisionPlane(PxVec3(0, 1, 0), 0.0f));
//		gCloth->addCollisionConvex(1 << 0); // Convex references the first plane
//
//#if PX_SUPPORT_GPU_PHYSX
//		gCloth->setClothFlag(PxClothFlag::eGPU, true);
//#endif
//
	}

	//4-Creating SPH Particle Fluid
	CreateFluid();






}


void StepPhysX()					//Stepping PhysX
{
	gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
	gScene->fetchResults(true);		//Block until the simulation run is completed
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



