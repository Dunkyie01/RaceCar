
// GamesAssignment2TDM.cpp: A program using the TL-Engine

//Made by Tyler Manning - 21090747 - Games Concepts Assignment 2 - CO1301

#include "TL-Engine11.h" 
#include <sstream>
#include <windows.h>
#include <string>
#include <iostream>
#include <cmath>


using namespace tle;
using namespace std;

/// Game play follows state model accurately (for classification implemented)
enum GameStates { Idle, StartSeq, Racing, StageOne, StageTwo, StageThree, FinishLine }; //enum to store state of game

// converts a game state to a string instead of a number
string GameStateToString(GameStates state)
{
	switch (state)
	{
	case Idle:       return "Idle";
	case StartSeq:   return "Start Sequence";
	case Racing:     return "Racing";
	case StageOne:   return "Stage One";
	case StageTwo:   return "Stage Two";
	case StageThree: return "Stage Three";
	case FinishLine: return "Finish Line";
	default:         return "Unknown";
	}
}

///2D vectors or 3D vectors are used to calculate car's momentum
// Vector maths for 3d plane
struct Vector3D
{
	float x;
	float y;
	float z;
};

// Vector maths for 2d plane
struct Vector2D
{
	float x;
	float z;
};

// Vector maths functions for 2D plane
Vector2D VectorAdd(Vector2D a, Vector2D b)
{
	Vector2D result;
	result.x = a.x + b.x;
	result.z = a.z + b.z;
	return result;
}
// Scales the vector 
Vector2D VectorScale(Vector2D v, float scalar)
{
	Vector2D result;
	result.x = v.x * scalar;
	result.z = v.z * scalar;
	return result;
}

// Subtracts one vector from another
Vector2D VectorSubtract(Vector2D a, Vector2D b)
{
	return VectorAdd(a, VectorScale(b, -1.f));
}

// Returns the length of a vector
float VectorLength(Vector2D v)
{
	return sqrtf(v.x * v.x + v.z * v.z);
}

// Clamps a value between a low and high value
float Clamp(float value, float lo, float hi)
{
	if (value < lo) return lo;
	if (value > hi) return hi;
	return value;
}

// Returns the minimum or maximum of two float values
float FloatMin(float a, float b) { return (a < b) ? a : b; }
float FloatMax(float a, float b) { return (a > b) ? a : b; }

// bounds box for AABB collision
struct AABB
{
	float minX, maxX;
	float minZ, maxZ;
};

// Creates an AABB from a centre point and half-width and half-depth
AABB MakeAABB(Vector2D centre, float halfWidth, float halfDepth)
{
	AABB box;
	box.minX = centre.x - halfWidth;
	box.maxX = centre.x + halfWidth;
	box.minZ = centre.z - halfDepth;
	box.maxZ = centre.z + halfDepth;
	return box;
}

// Merges two boxes into one box that contains both of them
AABB CombineAABB(AABB a, AABB b)
{
	AABB result;
	result.minX = FloatMin(a.minX, b.minX);
	result.maxX = FloatMax(a.maxX, b.maxX);
	result.minZ = FloatMin(a.minZ, b.minZ);
	result.maxZ = FloatMax(a.maxZ, b.maxZ);
	return result;
}

///Collision detection & resolution (sph-2-box isles & walls, sph2-sph struts of checkp, bouncing)
// Sphere-to-AABB collision (car vs walls, isles, small tanks)
bool SphereVsAABB(Vector2D sphereCentre, float radius, AABB box, Vector2D& pushOut)
{
	float closestX = Clamp(sphereCentre.x, box.minX, box.maxX);
	float closestZ = Clamp(sphereCentre.z, box.minZ, box.maxZ);

	Vector2D diff = VectorSubtract(sphereCentre, Vector2D{ closestX, closestZ });
	float distSq = diff.x * diff.x + diff.z * diff.z;

	if (distSq < radius * radius && distSq > 0.0001f)
	{
		float dist = sqrtf(distSq);
		float overlap = radius - dist;
		pushOut = VectorScale(diff, overlap / dist);
		return true;
	}
	return false;
}

// Sphere-to-sphere collision (car vs checkpoint struts)
bool SphereVsSphere(Vector2D centreA, float radiusA, Vector2D centreB, float radiusB, Vector2D& pushOut)
{
	Vector2D diff = VectorSubtract(centreA, centreB);
	float distSq = diff.x * diff.x + diff.z * diff.z;
	float minDist = radiusA + radiusB;

	if (distSq < minDist * minDist && distSq > 0.0001f)
	{
		float dist = sqrtf(distSq);
		float overlap = minDist - dist;
		pushOut = VectorScale(diff, overlap / dist);
		return true;
	}
	return false;
}

// Resolves a collision by moving the car out of the collision and adjusting its momentum instead of a dead stop
void ResolvePushOut(IModel* car, Vector2D pushOut, Vector2D& momentum)
{
	car->Move(pushOut.x, 0.f, pushOut.z);

	float len = VectorLength(pushOut);
	if (len > 0.0001f)
	{
		Vector2D normal = VectorScale(pushOut, 1.f / len);
		float inward = momentum.x * normal.x + momentum.z * normal.z;
		if (inward < 0.f)
		{
			momentum = VectorSubtract(momentum, VectorScale(normal, inward));
		}
	}
}

// Checks if a point is inside an AABB
bool PointInAABB(Vector2D point, AABB box)
{
	return point.x >= box.minX && point.x <= box.maxX &&
		point.z >= box.minZ && point.z <= box.maxZ;
}

// Creates an AABB for the gap between two checkpoint struts, with a given depth
AABB MakeCheckpointGapAABB(Vector2D strutA, Vector2D strutB, float gateHalfDepth)
{
	AABB box;
	box.minX = FloatMin(strutA.x, strutB.x);
	box.maxX = FloatMax(strutA.x, strutB.x);
	box.minZ = FloatMin(strutA.z, strutB.z);
	box.maxZ = FloatMax(strutA.z, strutB.z);

	float gapWidthX = box.maxX - box.minX;
	if (gapWidthX < 0.01f)
	{
		box.minX -= gateHalfDepth;
		box.maxX += gateHalfDepth;
	}
	else
	{
		box.minZ -= gateHalfDepth;
		box.maxZ += gateHalfDepth;
	}
	return box;
}

int main()
{
	// Create a 3D engine (using TL11 engine here) and open a window for it
	TLEngine* myEngine = New3DEngine(TL11);
	myEngine->StartFullscreen();

	// Add default folder for meshes and other media
	myEngine->AddMediaFolder(".\\Media");
	myEngine->AddMediaFolder("C:\\Users\\tyler\\Desktop\\RaceCar\\RaceCarAssignment 2\\Media");
	//myEngine->AddMediaFolder("C:\\Users\\Public\\Documents\\TL-Engine11\\Media");

	///Scene setup (positioning of objects (esp. checkpoints))
	/**** Set up your scene here ****/
	myEngine->Timer();
	IFont* myFont = myEngine->LoadFont("Calibri", 24);

	ISprite* hudBackdrop = myEngine->CreateSprite("ui_backdrop.png", 700, 60, 0);


	//Ground
	IMesh* groundMesh = myEngine->LoadMesh("ground.x");
	IModel* groundModel = groundMesh->CreateModel(0.f, 1.f, 0.f);
	//groundModel->SetSkin("Grid2.jpg");
	groundModel->SetSkin("sand.png");

	//Car
	IMesh* carMesh = myEngine->LoadMesh("race2.x");
	IModel* carModel = carMesh->CreateModel(0.f, 3.f, -30.f);
	//IModel* carModel = carMesh->CreateModel(0.f, 3.f, -10.f);
	carModel->SetSkin("sp02-01.jpg");   
	carModel->Scale(0.1f);

	//SkyBox
	IMesh* skyboxMesh = myEngine->LoadMesh("skybox 07.x");
	const Vector3D vSkyBoxCoords = { 0.f, -750.f, 0.f };
	IModel* skyboxModel = skyboxMesh->CreateModel(vSkyBoxCoords.x, vSkyBoxCoords.y, vSkyBoxCoords.z);

	//Camera
	const Vector3D vCameraCoords = { 0.f, 15.f, -65.f };
	ICamera* camera = myEngine->CreateCamera(kManual, vCameraCoords.x, vCameraCoords.y, vCameraCoords.z);
	camera->RotateLocalX(10.f);

	//chase camera
	camera->AttachToParent(carModel);
	bool chaseCam = true;

	//walls
	IMesh* wallMesh = myEngine->LoadMesh("Wall.x");
	const int WallCount = 4;
	IModel* wallArray[WallCount];
	wallArray[0] = wallMesh->CreateModel(-10, 0.f, 48.f);
	wallArray[1] = wallMesh->CreateModel(10, 0.f, 48.f);
	wallArray[2] = wallMesh->CreateModel(-10.f, 0.f, 64.f);
	wallArray[3] = wallMesh->CreateModel(10.f, 0.f, 64.f);

	//loop for skinning the walls
	for (int i = 0; i < WallCount; i++)
	{
		wallArray[i]->SetSkin("TribuneWall.jpg");
	}

	//Checkpoints
	IMesh* checkPtMesh = myEngine->LoadMesh("Checkpoint.obj");
	const int CheckPtCount = 4;
	IModel* checkPtArray[CheckPtCount];
	checkPtArray[0] = checkPtMesh->CreateModel(0.f, 0.f, 0.f);
	checkPtArray[1] = checkPtMesh->CreateModel(10.f, 0.f, 120.f);
	checkPtArray[1]->RotateLocalY(90.f);
	checkPtArray[2] = checkPtMesh->CreateModel(25.f, 0.f, 56.f);
	checkPtArray[3] = checkPtMesh->CreateModel(25.f, 0.f, 0.f);

	//loop for skinning the checkpoints
	for (int i = 0; i < CheckPtCount; i++)
	{
		checkPtArray[i]->SetSkin("TribuneWall.jpg");
	}

	//Pillars/isles
	IMesh* IsleMesh = myEngine->LoadMesh("IsleStraight.obj");
	const int IsleCount = 6;
	IModel* IsleArray[IsleCount];
	IsleArray[0] = IsleMesh->CreateModel(-10.f, 0.f, 40.f);
	IsleArray[1] = IsleMesh->CreateModel(10.f, 0.f, 40.f);
	IsleArray[2] = IsleMesh->CreateModel(-10.f, 0.f, 56.f);
	IsleArray[3] = IsleMesh->CreateModel(10.f, 0.f, 56.f);
	IsleArray[4] = IsleMesh->CreateModel(-10.f, 0.f, 72.f);
	IsleArray[5] = IsleMesh->CreateModel(10.f, 0.f, 72.f);

	//loop for skinning the pillars/isles
	for (int i = 0; i < IsleCount; i++)
	{
		IsleArray[i]->SetSkin("TribuneWall.jpg", 0);
		IsleArray[i]->SetSkin("TribuneWall.jpg", 1);
		IsleArray[i]->SetSkin("TribuneWall.jpg", 2);
	}

	//Large tanks for background - arranged in a ring around the circuit
//Large tanks for background - arranged in a ring around the circuit
	IMesh* tankMesh = myEngine->LoadMesh("TankLarge1.obj");
	const int TankCount = 13;
	IModel* tankArray[TankCount];
	tankArray[0] = tankMesh->CreateModel(105.f, 0.f, 60.f); //left point
	tankArray[1] = tankMesh->CreateModel(90.f, 0.f, 112.f);
	tankArray[2] = tankMesh->CreateModel(52.f, 0.f, -30.f);
	tankArray[3] = tankMesh->CreateModel(0.f, 0.f, 165.f);
	tankArray[4] = tankMesh->CreateModel(-52.f, 0.f, 150.f);
	tankArray[5] = tankMesh->CreateModel(-90.f, 0.f, 112.f);
	tankArray[6] = tankMesh->CreateModel(-105.f, 0.f, 60.f);
	tankArray[7] = tankMesh->CreateModel(-90.f, 0.f, 7.f);
	tankArray[8] = tankMesh->CreateModel(-52.f, 0.f, -30.f);
	tankArray[9] = tankMesh->CreateModel(0.f, 0.f, -55.f);
	tankArray[10] = tankMesh->CreateModel(52.f, 0.f, -30.f);
	tankArray[11] = tankMesh->CreateModel(90.f, 0.f, 7.f);
	tankArray[12] = tankMesh->CreateModel(52.f, 0.f, 150.f); 

	//loop for skinning Large tanks
	for (int i = 0; i < TankCount; i++)
	{
		tankArray[i]->SetSkin("TankSpec.jpg", 2);
		tankArray[i]->SetSkin("Tank2.jpg", 1);
	}

	//loop for skinning Large tanks
	for (int i = 0; i < TankCount; i++)
	{
		tankArray[i]->SetSkin("TankSpec.jpg", 2);
		tankArray[i]->SetSkin("Tank2.jpg", 1);
	}

	//small tanks for track definition
	IMesh* SmallTankMesh = myEngine->LoadMesh("TankSmall1.obj");
	const int SmallTankCount = 26;
	IModel* SmallTankArray[SmallTankCount];
	//curve 1
	SmallTankArray[0] = SmallTankMesh->CreateModel(-10.f, 1.f, 90.f); //first tank curve 1
	SmallTankArray[1] = SmallTankMesh->CreateModel(-10.f, 1.f, 100.f);
	SmallTankArray[2] = SmallTankMesh->CreateModel(-10.f, 1.f, 110.f);
	SmallTankArray[3] = SmallTankMesh->CreateModel(-6.f, 1.f, 120.f);
	SmallTankArray[4] = SmallTankMesh->CreateModel(3.f, 1.f, 127.f);
	SmallTankArray[5] = SmallTankMesh->CreateModel(10.f, 1.f, 130.f); // last tank curve 1
	//curve 2
	SmallTankArray[6] = SmallTankMesh->CreateModel(10.f, 1.f, 130.f); //first tank curve 2
	SmallTankArray[7] = SmallTankMesh->CreateModel(20.f, 1.f, 130.f);
	SmallTankArray[8] = SmallTankMesh->CreateModel(30.f, 1.f, 127.f);
	SmallTankArray[9] = SmallTankMesh->CreateModel(36.f, 1.f, 120.f);
	SmallTankArray[10] = SmallTankMesh->CreateModel(40.f, 1.f, 110.f);
	SmallTankArray[11] = SmallTankMesh->CreateModel(40.f, 1.f, 100.f);
	SmallTankArray[12] = SmallTankMesh->CreateModel(40.f, 1.f, 90.f); //last tank curve 2
	//behind start line
	SmallTankArray[13] = SmallTankMesh->CreateModel(-10.f, 1.f, -50.f); //far left
	SmallTankArray[14] = SmallTankMesh->CreateModel(-5.f, 1.f, -50.f);
	SmallTankArray[15] = SmallTankMesh->CreateModel(0.f, 1.f, -50.f); //centre
	SmallTankArray[16] = SmallTankMesh->CreateModel(5.f, 1.f, -50.f);
	SmallTankArray[17] = SmallTankMesh->CreateModel(10.f, 1.f, -50.f); //far right

	//start line borders
	//left side
	SmallTankArray[18] = SmallTankMesh->CreateModel(-10.f, 1.f, -40.f);
	SmallTankArray[19] = SmallTankMesh->CreateModel(-10.f, 1.f, -30.f);
	SmallTankArray[20] = SmallTankMesh->CreateModel(-10.f, 1.f, -20.f);
	SmallTankArray[21] = SmallTankMesh->CreateModel(-10.f, 1.f, -10.f);
	//right side
	SmallTankArray[22] = SmallTankMesh->CreateModel(10.f, 1.f, -40.f); //far right
	SmallTankArray[23] = SmallTankMesh->CreateModel(10.f, 1.f, -30.f); //far right
	SmallTankArray[24] = SmallTankMesh->CreateModel(10.f, 1.f, -20.f); //far right
	SmallTankArray[25] = SmallTankMesh->CreateModel(10.f, 1.f, -10.f); //far right

	//loop for skinning Small tanks
	for (int i = 0; i < SmallTankCount; i++)
	{
		SmallTankArray[i]->SetSkin("Tank2.jpg", 0);
		SmallTankArray[i]->SetSkin("TankSpec.jpg", 1);

		SmallTankArray[i]->Scale(0.25f);
	}

	//Collision box geometry

	const float carRadius = 3.f;             // radius of sphere for collision
	const float isleHalfWidth = 2.f;
	const float isleHalfDepth = 8.f;
	const float wallHalfWidth = 2.f;
	const float wallHalfDepth = 4.f;
	const float smallTankHalfWidth = 2.f;
	const float smallTankHalfDepth = 4.f;


	// Combine the AABBs for the isles and walls into a single AABB for each barrier
	AABB leftBarrier1 = CombineAABB(CombineAABB(
		MakeAABB(Vector2D{ IsleArray[0]->GetX(), IsleArray[0]->GetZ() }, isleHalfWidth, isleHalfDepth),
		MakeAABB(Vector2D{ wallArray[0]->GetX(), wallArray[0]->GetZ() }, wallHalfWidth, wallHalfDepth)),
		MakeAABB(Vector2D{ IsleArray[2]->GetX(), IsleArray[2]->GetZ() }, isleHalfWidth, isleHalfDepth));

	AABB rightBarrier1 = CombineAABB(CombineAABB(
		MakeAABB(Vector2D{ IsleArray[1]->GetX(), IsleArray[1]->GetZ() }, isleHalfWidth, isleHalfDepth),
		MakeAABB(Vector2D{ wallArray[1]->GetX(), wallArray[1]->GetZ() }, wallHalfWidth, wallHalfDepth)),
		MakeAABB(Vector2D{ IsleArray[3]->GetX(), IsleArray[3]->GetZ() }, isleHalfWidth, isleHalfDepth));

	AABB leftBarrier2 = CombineAABB(CombineAABB(
		MakeAABB(Vector2D{ IsleArray[2]->GetX(), IsleArray[2]->GetZ() }, isleHalfWidth, isleHalfDepth),
		MakeAABB(Vector2D{ wallArray[2]->GetX(), wallArray[2]->GetZ() }, wallHalfWidth, wallHalfDepth)),
		MakeAABB(Vector2D{ IsleArray[4]->GetX(), IsleArray[4]->GetZ() }, isleHalfWidth, isleHalfDepth));

	AABB rightBarrier2 = CombineAABB(CombineAABB(
		MakeAABB(Vector2D{ IsleArray[3]->GetX(), IsleArray[3]->GetZ() }, isleHalfWidth, isleHalfDepth),
		MakeAABB(Vector2D{ wallArray[3]->GetX(), wallArray[3]->GetZ() }, wallHalfWidth, wallHalfDepth)),
		MakeAABB(Vector2D{ IsleArray[5]->GetX(), IsleArray[5]->GetZ() }, isleHalfWidth, isleHalfDepth));

	const int BarrierCount = 4;
	AABB barriers[BarrierCount] = { leftBarrier1, rightBarrier1, leftBarrier2, rightBarrier2 };

	//checkpoint struts
	const float strutHalfGap = 4.f;
	const float strutRadius = 1.f;
	float checkPtRotationY[CheckPtCount] = { 0.f, 90.f, 0.f, 0.f };
	const float gateHalfDepth = 3.f;  

	// checks for correct dir of travel
	float checkPtForwardSign[CheckPtCount] = { 1.f, 1.f, -1.f, -1.f };

	//Working out where the struts are and the gap for collision 
	Vector2D checkPtStrutA[CheckPtCount];
	Vector2D checkPtStrutB[CheckPtCount];
	AABB checkPtGapBox[CheckPtCount];
	Vector2D checkPtForward[CheckPtCount];
	for (int i = 0; i < CheckPtCount; i++)
	{
		float rotRad = checkPtRotationY[i] * 3.14159265f / 180.f;
		Vector2D right = { cosf(rotRad), -sinf(rotRad) };
		Vector2D centre = { checkPtArray[i]->GetX(), checkPtArray[i]->GetZ() };
		checkPtStrutA[i] = VectorAdd(centre, VectorScale(right, strutHalfGap));
		checkPtStrutB[i] = VectorAdd(centre, VectorScale(right, -strutHalfGap));
		checkPtGapBox[i] = MakeCheckpointGapAABB(checkPtStrutA[i], checkPtStrutB[i], gateHalfDepth);

		checkPtForward[i] = Vector2D{ sinf(rotRad) * checkPtForwardSign[i], cosf(rotRad) * checkPtForwardSign[i] };
	}

	//small tank collision
	AABB smallTankCollision[SmallTankCount];

	for (int i = 0; i < SmallTankCount; i++)
	{
		smallTankCollision[i] = MakeAABB(
			Vector2D{
				SmallTankArray[i]->GetX(),
				SmallTankArray[i]->GetZ()
			},
			smallTankHalfWidth,
			smallTankHalfDepth
		);
	}

	// check for passing a chackpoint
	const float checkpointTriggerRadius = 6.f;
	int nextCheckpoint = 0;


	//main variable list

	//car direction
	bool CarForeward = true;
	float currentZ = 0.0f;

	//mouse capture
	bool mouseCaptureActive = true;
	myEngine->StartMouseCapture();
	float mouseMoveX = 0.f;
	float mouseMoveY = 0.f;
	float cameraYaw = 0.0f;
	float cameraPitch = 0.0f;

	// Game state
	GameStates currentState = Idle;
	bool raceTimerRunning = false;
	float raceTime = 0.f;

	//timer
	myEngine->Timer();
	float countdown = 0.f;
	string dialogueText = "Press Space to Start";

	//*cspeed constant*
	const float cSpeed = 0.1f;
	const float MetreScale = 1.f; // 1 unit = 1 metre
	///Demo runs at a playable speed; frame time is used to adjust speed
	const float frameTimeCap = 0.1f; // cap for frame time

	//car movement 
	float thrustforce = 0.2f; // acceleration
	float dragCoeff = 0.95f;      //drag when thrusting
	float coastDragCoeff = 0.9f;  // drag when not thrusting
	Vector2D momentum = { 0.f, 0.f }; //momentum vector  
	const float maxSpeed = 2.f; // max speed cap
	float currentSpeed = 0.f;     // current speed of the car

	float carHeading = 0.f;       // car facing
	const float turnRate = 2.f;	// turn rate 

	//camera limits
	const float cYawLimit = 115.0f; //Yaw limit (degs)
	const float cPitchLimit = 45.0f; //Pitch limit (degs)
	const float cReturnSpeed = 0.1f; //Speed camera resets to centre
	const float cMouseWheelMove = 0.1f; //Mouse wheel movement sensitivity
	const float cMouseRotation = 0.1f; //Mouse rotation sensitivity


	 
	// The main game loop, repeat until engine is stopped
	while (myEngine->IsRunning())
	{
		// Draw the scene
		myEngine->DrawScene();
		float frameTime = myEngine->Timer();
		//Limiting frame time to avoid physics issues
		if (frameTime > frameTimeCap)
		{
			frameTime = frameTimeCap;
		}


		/**** Update your scene each frame here ****/


		//Game text

		myFont->Draw("Current State is " + GameStateToString(currentState), 5, 5, kWhite); ///Display the current state of the game in backdrop (use sprite)
		myFont->Draw("Race Time is " + std::to_string(raceTime/100), 5, 80, kWhite);
		myFont->Draw("Press 1 to reset orientation", 5, 135, kWhite);
		myFont->Draw("Press Tab to toggle chase cam", 5, 160, kWhite);
		myFont->Draw("Press Z to show debug info", 5, 185, kWhite);
		myFont->Draw("Press T to view controls", 5, 210, kWhite);


		myFont->Draw("Speed: " + std::to_string(static_cast<int>(std::round(currentSpeed))) + " m/s", 5, 440, kWhite);
		myFont->Draw("Next Checkpoint is " + std::to_string(nextCheckpoint), 5, 465, kWhite);

		//controls
		if (myEngine->KeyHit(Key_T))
		{
			myFont->Draw("Controls:", 5, 330, kWhite);
			myFont->Draw("W = Forward", 5, 360, kWhite);
			myFont->Draw("S = Backward", 5, 385, kWhite);
			myFont->Draw("A = Rotate Left", 5, 410, kWhite);
			myFont->Draw("D = Rotate Right", 5, 435, kWhite);
		}
		//debug
		if (myEngine->KeyHit(Key_L))
		{
			currentState = Racing;
			raceTimerRunning = true;
			raceTime = 0.f;
		}
		if (myEngine->KeyHeld(Key_Z))
		{
			myFont->Draw("Mouse sensitivity is " + std::to_string(cMouseRotation), 5, 230, kWhite);
			myFont->Draw("Mouse move X is " + std::to_string(mouseMoveX), 5, 310, kWhite);
			myFont->Draw("Mouse move Y is " + std::to_string(mouseMoveY), 5, 280, kWhite);
			myFont->Draw("Screen Size is " + std::to_string(myEngine->GetWidth()) + " x " + std::to_string(myEngine->GetHeight()), 5, 500, kWhite);
			myFont->Draw("Momentum X/Z is " + std::to_string(momentum.x) + " / " + std::to_string(momentum.z), 5, 335, kWhite);
			myFont->Draw("Car Heading is " + std::to_string(carHeading), 5, 360, kWhite);
			myFont->Draw("Car X is " + std::to_string(carModel->GetX()), 5, 255, kWhite);
			myFont->Draw("Car Y is " + std::to_string(carModel->GetY()), 5, 280, kWhite);
			myFont->Draw("Car Z is " + std::to_string(carModel->GetZ()), 5, 305, kWhite);
			myFont->Draw("Frame time is " + std::to_string(frameTime), 5, 5, kWhite);

		}

		//Start
		if (currentState == Idle)
		{
			myFont->Draw("Press Space to Start", myEngine->GetWidth() / 2 - 150, myEngine->GetHeight() / 2 + 400, kWhite);
		}
		else
		{
			myFont->Draw(dialogueText, myEngine->GetWidth() / 2 - 150, myEngine->GetHeight() / 2 + 400, kWhite);
		}

		if (myEngine->KeyHit(Key_Space))
		{
			countdown = 0.f;
			currentState = StartSeq;
		}
		///Dialog "Hit Space to Start" and countdown
		if (currentState == StartSeq) 
		{
			countdown += frameTime;

			if (countdown < 60.0f)
			{
				dialogueText = "3";
			}
			else if (countdown < 120.0f)
			{
				dialogueText = "2";
			}
			else if (countdown < 180.0f)
			{
				dialogueText = "1";
			}
			else if (countdown < 240.0f)
			{
				dialogueText = "GO!";
				raceTimerRunning = true;
				raceTime = 0.f;
			}
			else
			{
				dialogueText = "Stage 0";
				currentState = Racing;
				raceTimerRunning = true;
				raceTime = 0.f;
			}
		}

		//timer for the race
		if (raceTimerRunning == true)
		{
			raceTime += frameTime;
		}

		//controls for the game, active only when the game is in a racing state
		if (currentState == Racing || currentState == StageOne || currentState == StageTwo || currentState == StageThree)
		{
			///Turning of car clockwise/anticlockwise (DA keys)
			//steering controls left and right
			if (myEngine->KeyHeld(Key_A))
			{
				carHeading -= turnRate * cSpeed;
				carModel->RotateLocalY(-turnRate * cSpeed);
			}
			if (myEngine->KeyHeld(Key_D))
			{
				carHeading += turnRate * cSpeed;
				carModel->RotateLocalY(turnRate * cSpeed);
			}

			//reset orientation
			if (carHeading > 360.f) carHeading -= 360.f;
			if (carHeading < -360.f) carHeading += 360.f;

			///Forward/backward of hovercar (WS keys), max backwards = 50 % of max forwardthrust
			//thrust to the car
			float headingRad = carHeading * 3.14159265f / 180.f;
			Vector2D thrust = { 0.f, 0.f };

			if (myEngine->KeyHeld(Key_W))
			{
				thrust = Vector2D{ sinf(headingRad) * thrustforce, cosf(headingRad) * thrustforce };
			}
			if (myEngine->KeyHeld(Key_S))
			{
				thrust = Vector2D{ -sinf(headingRad) * thrustforce * 0.5f, -cosf(headingRad) * thrustforce * 0.5f };
			}
			if (myEngine->KeyHeld(Key_W) && myEngine->KeyHeld(Key_S))
			{
				thrust = Vector2D{ 0.f, 0.f };
			}

			///2D vectors or 3D vectors are used to calculate car's momentum
			//combining thrust with momentum
			momentum = VectorAdd(momentum, VectorScale(thrust, cSpeed));
			bool isThrusting = myEngine->KeyHeld(Key_W) || myEngine->KeyHeld(Key_S);
			float dragPerSecond = isThrusting ? dragCoeff : coastDragCoeff;
			float appliedDrag = powf(dragPerSecond, cSpeed);
			momentum = VectorScale(momentum, appliedDrag);

			//speed cap
			if (VectorLength(momentum) > maxSpeed)
			{
				momentum = VectorScale(momentum, maxSpeed / VectorLength(momentum));
			}
			currentSpeed = VectorLength(momentum);

			//moving the car each frame
			carModel->Move(momentum.x * cSpeed, 0.f, momentum.z * cSpeed);

			//calc for current speed
			currentSpeed = VectorLength(momentum);

			currentZ = carModel->GetZ();
			if (currentZ >= 10.f)
			{
				CarForeward = false;
			}
			if (currentZ <= -10.f)
			{
				CarForeward = true;
			}

			//stops the car at the finish
			if (currentState == FinishLine) ///Dialog "Stage X complete" and "Race complete
			{
				momentum = { 0.f,0.f };
				myFont->Draw("Race Time is " + std::to_string(raceTime / 100) + " Seconds", myEngine->GetWidth() / 2 , myEngine->GetHeight() / 2 , kWhite);
			}

			///Collision detection & resolution (sph2-box isles & walls, sph2-sph struts of checkp, bouncing
			//collision for car and isle
			Vector2D carPos = { carModel->GetX(), carModel->GetZ() };
			for (int i = 0; i < BarrierCount; i++)
			{
				Vector2D pushOut;
				if (SphereVsAABB(carPos, carRadius, barriers[i], pushOut))
				{
					ResolvePushOut(carModel, pushOut, momentum);
					carPos = Vector2D{ carModel->GetX(), carModel->GetZ() };
				}
			}

			//collision for car and small tanks
			for (int i = 0; i < SmallTankCount; i++)
			{
				Vector2D pushOut;

				if (SphereVsAABB(carPos, carRadius, smallTankCollision[i], pushOut))
				{
					ResolvePushOut(carModel, pushOut, momentum);

					carPos = Vector2D{
						carModel->GetX(),
						carModel->GetZ()
					};
				}
			}

			//collision for car and checkpoint struts, and checkpoint trigger
			for (int i = 0; i < CheckPtCount; i++)
			{
				Vector2D pushOut;
				if (SphereVsSphere(carPos, carRadius, checkPtStrutA[i], strutRadius, pushOut))
				{
					ResolvePushOut(carModel, pushOut, momentum);
					carPos = Vector2D{ carModel->GetX(), carModel->GetZ() };
				}
				if (SphereVsSphere(carPos, carRadius, checkPtStrutB[i], strutRadius, pushOut))
				{
					ResolvePushOut(carModel, pushOut, momentum);
					carPos = Vector2D{ carModel->GetX(), carModel->GetZ() };
				}
				
				// Check if the car is passing through the checkpoint gap also checks if the car is moving the right way
				if (i == nextCheckpoint && PointInAABB(carPos, checkPtGapBox[i]))
				{
					float approachDir = momentum.x * checkPtForward[i].x
						+ momentum.z * checkPtForward[i].z;

					// If the car is moving in the right direction, advance to the next checkpoint
					if (approachDir > 0.f)
					{
						nextCheckpoint++;

						switch (nextCheckpoint)
						{
						case 1:
							dialogueText = "Stage 1 Complete";
							currentState = StageOne;
							break;

						case 2:
							dialogueText = "Stage 2 Complete";
							currentState = StageTwo;
							break;

						case 3:
							dialogueText = "Stage 3 Complete";
							currentState = StageThree;
							break;

						case 4:
							dialogueText = "Race Complete! Your time was " + std::to_string(raceTime / 100);
							currentState = FinishLine;
							raceTimerRunning = false;
							break;
						}
					}
				}
			}

		}
		///Chase cam behaviour (arrow keys, mouse movement, reset of cam position above/behind)
		//Camera Movement

		//Chase cam toggle
		if (myEngine->KeyHit(Key_Tab))
		{
			chaseCam = !chaseCam;

			if (chaseCam == false)
			{
				// Free cam
				camera->DetachFromParent();
				camera->ResetOrientation();
				camera->SetPosition(vCameraCoords.x, vCameraCoords.y, vCameraCoords.z);
				camera->RotateLocalX(10.f);
				cameraYaw = 0.0f;
				cameraPitch = 0.0f;
			}
			else
			{
				camera->AttachToParent(carModel);
				camera->ResetOrientation();
				camera->RotateLocalX(10.f);
			}

		}
		//Reset Camera Pos
		if (myEngine->KeyHit(Key_1))
		{
			camera->ResetOrientation();
			cameraYaw = 0.0f;
			cameraPitch = 0.0f;
		}
		// Keyboard controlled movement
		if (!chaseCam)
		{
			if (myEngine->KeyHeld(Key_Up))
			{
				camera->MoveLocalZ(cMouseRotation * cSpeed);
			}
			if (myEngine->KeyHeld(Key_Down))
			{
				camera->MoveLocalZ(cMouseRotation * -cSpeed);
			}
			if (myEngine->KeyHeld(Key_Right))
			{
				camera->RotateLocalY(cMouseRotation * cSpeed);
			}
			if (myEngine->KeyHeld(Key_Left))
			{
				camera->RotateLocalY(cMouseRotation * -cSpeed);
			}
		}

		// Mouse controlled movement
		if (mouseCaptureActive)
		{
			mouseMoveX = myEngine->GetMouseMovementX();
			mouseMoveY = myEngine->GetMouseMovementY();

			cameraYaw += mouseMoveX * cMouseRotation;
			cameraPitch += mouseMoveY * cMouseRotation;

			if (cameraYaw > cYawLimit)   cameraYaw = cYawLimit;
			if (cameraYaw < -cYawLimit)   cameraYaw = -cYawLimit;
			if (cameraPitch > cPitchLimit) cameraPitch = cPitchLimit;
			if (cameraPitch < -cPitchLimit) cameraPitch = -cPitchLimit;
		}
		else
		{
			cameraYaw += (0.0f - cameraYaw) * cReturnSpeed * cSpeed;
			cameraPitch += (0.0f - cameraPitch) * cReturnSpeed * cSpeed;
		}

		// Mouse wheel movement
		float mouseWheelMove = myEngine->GetMouseWheelMovement();
		camera->MoveLocalZ(mouseWheelMove * cMouseWheelMove);

		//Camera rotation and limits
		if (chaseCam)
		{
			// Locked Cam
			if (mouseMoveX != 0 || mouseMoveY != 0)
			{
				camera->RotateLocalY(mouseMoveX * cMouseRotation);
				camera->RotateLocalX(-mouseMoveY * cMouseRotation);
			}

			if (cameraYaw > cYawLimit)
			{
				cameraYaw = cYawLimit;
			}
			if (cameraYaw < -cYawLimit)
			{
				cameraYaw = -cYawLimit;
			}
			if (cameraPitch > cPitchLimit)
			{
				cameraPitch = cPitchLimit;
			}
			if (cameraPitch < -cPitchLimit)
			{
				cameraPitch = -cPitchLimit;
			}

		}
		camera->ResetOrientation();
		camera->RotateLocalY(cameraYaw);
		camera->RotateLocalX(cameraPitch);

		// Stop if the Escape key is pressed
		if (myEngine->KeyHit(Key_Escape))
		{
			myEngine->Stop();
		}

	}

	// Delete the 3D engine now we are finished with it
	myEngine->Delete();
}