package org.gearvrf.bulletsample;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.gearvrf.FutureWrapper;
import org.gearvrf.GVRAndroidResource;
import org.gearvrf.GVRCameraRig;
import org.gearvrf.GVRContext;
import org.gearvrf.GVRMesh;
import org.gearvrf.GVRScene;
import org.gearvrf.GVRSceneObject;
import org.gearvrf.GVRScript;
import org.gearvrf.bulletphysics.collision.broadphase.BroadphaseInterface;
import org.gearvrf.bulletphysics.collision.broadphase.BroadphaseNativeType;
import org.gearvrf.bulletphysics.collision.broadphase.DbvtBroadphase;
import org.gearvrf.bulletphysics.collision.dispatch.CollisionDispatcher;
import org.gearvrf.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import org.gearvrf.bulletphysics.collision.shapes.BoxShape;
import org.gearvrf.bulletphysics.collision.shapes.SphereShape;
import org.gearvrf.bulletphysics.collision.shapes.StaticPlaneShape;
import org.gearvrf.bulletphysics.dynamics.DiscreteDynamicsWorld;
import org.gearvrf.bulletphysics.dynamics.DynamicsWorld;
import org.gearvrf.bulletphysics.dynamics.RigidBody;
import org.gearvrf.bulletphysics.dynamics.RigidBodyConstructionInfo;
import org.gearvrf.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import org.gearvrf.bulletphysics.linearmath.DefaultMotionState;
import org.gearvrf.bulletphysics.linearmath.Transform;

import android.graphics.Color;

public class BulletSampleViewManager extends GVRScript {
    private static final String TAG = BulletSampleViewManager.class.getSimpleName();

    private GVRContext mGVRContext = null;
    private DynamicsWorld dynamicsWorld;

    private Map<RigidBody, GVRSceneObject> rigidBodiesSceneMap = new HashMap<RigidBody, GVRSceneObject>();

    private static final float CUBE_MASS = 0.5f;
    private static final float BALL_HEIGHT = 100f;

    private Transform transform = new Transform();

    private GVRScene scene;

    @Override
    public void onInit(GVRContext gvrContext) {
        mGVRContext = gvrContext;

        scene = mGVRContext.getNextMainScene();

        GVRCameraRig mainCameraRig = scene.getMainCameraRig();
        mainCameraRig.getLeftCamera().setBackgroundColor(Color.BLACK);
        mainCameraRig.getRightCamera().setBackgroundColor(Color.BLACK);

        mainCameraRig.getTransform().setPosition(0.0f, 6.0f, 0.0f);

        /*
         * Init bullet
         */
        BroadphaseInterface broadphase = new DbvtBroadphase();
        DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
        CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
        SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();

        /*
         * Create the physics world.
         */
        dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

        // set the gravity of our world
        dynamicsWorld.setGravity(new Vector3f(0, -10f, 0));

        /*
         * Create the ground. A simple textured quad. In bullet it will be a
         * plane shape with 0 mass
         */
        GVRSceneObject groundScene = quadWithTexture(100.0f, 100.0f,
                "floor.jpg");
        groundScene.getTransform().setRotationByAxis(-90.0f, 1.0f, 0.0f, 0.0f);
        groundScene.getTransform().setPosition(0.0f, 0.0f, 0.0f);
        scene.addSceneObject(groundScene);

        StaticPlaneShape floorShape = new StaticPlaneShape(new Vector3f(0.0f,
                1.0f, 0.0f), 0.0f);

        // setup the motion state
        DefaultMotionState floorState = new DefaultMotionState(
                new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 0, 0), 1.0f)));

        RigidBodyConstructionInfo floorRigidBodyCI = new RigidBodyConstructionInfo(0 /* mass */,
                floorState, floorShape, new Vector3f(0, 0, 0) /* inertia */);

        RigidBody floorRigidBody = new RigidBody(floorRigidBodyCI);

        dynamicsWorld.addRigidBody(floorRigidBody);

        /*
         * Create Some cubes in Bullet world and hit it with a sphere
         */
        addCube(scene, 0.0f, 1.0f, -9.0f, CUBE_MASS);
        addCube(scene, 0.0f, 1.0f, -10.0f, CUBE_MASS);
        addCube(scene, 0.0f, 1.0f, -11.0f, CUBE_MASS);
        addCube(scene, 1.0f, 1.0f, -9.0f, CUBE_MASS);
        addCube(scene, 1.0f, 1.0f, -10.0f, CUBE_MASS);
        addCube(scene, 1.0f, 1.0f, -11.0f, CUBE_MASS);
        addCube(scene, 2.0f, 1.0f, -9.0f, CUBE_MASS);
        addCube(scene, 2.0f, 1.0f, -10.0f, CUBE_MASS);
        addCube(scene, 2.0f, 1.0f, -11.0f, CUBE_MASS);

        addCube(scene, 0.0f, 2.0f, -9.0f, CUBE_MASS);
        addCube(scene, 0.0f, 2.0f, -10.0f, CUBE_MASS);
        addCube(scene, 0.0f, 2.0f, -11.0f, CUBE_MASS);
        addCube(scene, 1.0f, 2.0f, -9.0f, CUBE_MASS);
        addCube(scene, 1.0f, 2.0f, -10.0f, CUBE_MASS);
        addCube(scene, 1.0f, 2.0f, -11.0f, CUBE_MASS);
        addCube(scene, 2.0f, 2.0f, -9.0f, CUBE_MASS);
        addCube(scene, 2.0f, 2.0f, -10.0f, CUBE_MASS);
        addCube(scene, 2.0f, 2.0f, -11.0f, CUBE_MASS);

        addCube(scene, 0.0f, 3.0f, -9.0f, CUBE_MASS);
        addCube(scene, 0.0f, 3.0f, -10.0f, CUBE_MASS);
        addCube(scene, 0.0f, 3.0f, -11.0f, CUBE_MASS);
        addCube(scene, 1.0f, 3.0f, -9.0f, CUBE_MASS);
        addCube(scene, 1.0f, 3.0f, -10.0f, CUBE_MASS);
        addCube(scene, 1.0f, 3.0f, -11.0f, CUBE_MASS);
        addCube(scene, 2.0f, 3.0f, -9.0f, CUBE_MASS);
        addCube(scene, 2.0f, 3.0f, -10.0f, CUBE_MASS);
        addCube(scene, 2.0f, 3.0f, -11.0f, CUBE_MASS);

        /*
         * Throw a sphere from top
         */
        addSphere(scene, 1.0f, 1.5f, BALL_HEIGHT, -10.0f, 20.0f);
    }

    int cnt = 0;
    DefaultMotionState sphereState_;
    Transform sphereOrigin_;

    @Override
    public void onStep() {
        dynamicsWorld.stepSimulation(1.0f / 60.f, 10);
        for (RigidBody body : rigidBodiesSceneMap.keySet()) {
            if (body.getCollisionShape().getShapeType() == BroadphaseNativeType.SPHERE_SHAPE_PROXYTYPE
                    || body.getCollisionShape().getShapeType() == BroadphaseNativeType.BOX_SHAPE_PROXYTYPE) {
                body.getMotionState().getWorldTransform(transform);
                rigidBodiesSceneMap
                .get(body)
                .getTransform()
                .setPosition(
                        transform.origin.x,
                        transform.origin.y,
                        transform.origin.z);
            }
        }
    }

    private GVRSceneObject quadWithTexture(float width, float height,
            String texture) {
        FutureWrapper<GVRMesh> futureMesh = new FutureWrapper<GVRMesh>(
                mGVRContext.createQuad(width, height));
        GVRSceneObject object = null;
        try {
            object = new GVRSceneObject(mGVRContext, futureMesh,
                    mGVRContext.loadFutureTexture(new GVRAndroidResource(
                            mGVRContext, texture)));
        } catch (IOException e) {
            e.printStackTrace();
        }
        return object;
    }

    private GVRSceneObject meshWithTexture(String mesh, String texture) {
        GVRSceneObject object = null;
        try {
            object = new GVRSceneObject(mGVRContext, new GVRAndroidResource(
                    mGVRContext, mesh), new GVRAndroidResource(mGVRContext,
                    texture));
        } catch (IOException e) {
            e.printStackTrace();
        }
        return object;
    }

    /*
     * Function to add a cube of unit size with mass at the specified position
     * in Bullet physics world and scene graph.
     */
    private void addCube(GVRScene scene, float x, float y, float z, float mass) {
        BoxShape boxShape = new BoxShape(new Vector3f(0.5f, 0.5f, 0.5f));

        DefaultMotionState boxState = new DefaultMotionState(
                new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(x, y, z), 1.0f)));

        RigidBodyConstructionInfo boxRigidBodyCI =
                new RigidBodyConstructionInfo(mass, boxState, boxShape, new Vector3f(0, 0, 0));
        RigidBody boxRigidBody = new RigidBody(boxRigidBodyCI);
        dynamicsWorld.addRigidBody(boxRigidBody);

        GVRSceneObject cubeObject = meshWithTexture("cube.obj", "cube.jpg");

        cubeObject.getTransform().setPosition(x, y, z);
        scene.addSceneObject(cubeObject);
        rigidBodiesSceneMap.put(boxRigidBody, cubeObject);
    }

    /*
     * Function to add a sphere of dimension and position specified in the
     * Bullet physics world and scene graph
     */
    private void addSphere(GVRScene scene, float radius, float x, float y,
            float z, float mass) {
        SphereShape sphereShape = new SphereShape(radius);
        DefaultMotionState sphereState = new DefaultMotionState(
                sphereOrigin_ = new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(x, y, z), 1.0f)));
        sphereState_ = sphereState;

        RigidBodyConstructionInfo sphereRigidBodyCI =
                new RigidBodyConstructionInfo(mass, sphereState, sphereShape, new Vector3f(0, 0, 0));
        RigidBody sphereRigidBody = new RigidBody(sphereRigidBodyCI);
        dynamicsWorld.addRigidBody(sphereRigidBody);

        GVRSceneObject sphereObject = meshWithTexture("sphere.obj",
                "sphere.jpg");

        sphereObject.getTransform().setPosition(x, y, z);
        scene.addSceneObject(sphereObject);
        rigidBodiesSceneMap.put(sphereRigidBody, sphereObject);
    }
}
