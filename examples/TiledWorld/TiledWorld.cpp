/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "TiledWorld.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 10
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Z 200

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct TiledWorld : public CommonRigidBodyBase
{
	TiledWorld(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~TiledWorld() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation(float deltaTime) override;
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void TiledWorld::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		float a = 0.1f;

		btBoxShape* cube = createBoxShape(btVector3(a, a, a));
		m_collisionShapes.push_back(cube);

		/// Create Dynamic Objects
		btTransform transform;
		transform.setIdentity();

		// Ground
		{
			btScalar mass(0.0);
			for (int k = 0; k < 10; k++)
			{
				for (int j = 0; j < 200; j++)
				{
					transform.setOrigin(btVector3(
						0,
						btScalar(-a - 2.0 * a * k),
						btScalar(2.0 * a * j - (ARRAY_SIZE_Z - 1) * a)));

					createRigidBody(mass, transform, cube, btVector4(0, 1, 0, 1));
				}
			}
		}

		// Pyramid
		{
			const int e_count = 20;
			btScalar mass(1.0);
			btVector3 x(0.0f, a, -(e_count - 1) * a);
			btVector3 deltaX(0.0f, 2.0f * a, a);
			btVector3 deltaY(0.0f, 0.0f, 2.0f * a);

			for (int i = 0; i < e_count; ++i)
			{
				btVector3 y = x;

				for (int j = i; j < e_count; ++j)
				{
					transform.setOrigin(y);
					createRigidBody(mass, transform, cube);
					y += deltaY;
				}

				x += deltaX;
			}
		}

		m_dynamicsWorld->getBroadphase()->printStats();
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TiledWorld::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void TiledWorld::stepSimulation(float dt)
{
	CommonRigidBodyBase::stepSimulation(dt);
	btBroadphaseInterface* broadphase = m_dynamicsWorld->getBroadphase();
	//m_broadphase->optimizeIncremental(100);
	m_broadphase->printStats();
}

CommonExampleInterface* TiledWorldCreateFunc(CommonExampleOptions& options)
{
	return new TiledWorld(options.m_guiHelper);
}
