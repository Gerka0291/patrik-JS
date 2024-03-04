import {
    WebGLRenderer,
    PerspectiveCamera,
    Color,
    Scene,
    DirectionalLight,
    AmbientLight,
    Group,
    Vector3,
    Mesh,
    BoxGeometry,
    XRHandJoint,
    SphereBufferGeometry,
    MeshBasicMaterial,
    MeshPhongMaterial,
    SphereGeometry,
    Vector2,
    Box3,
    HemisphereLight,
    BufferGeometry,
    Matrix4,
    Quaternion


} from 'three';
import {
    Solver,
    Link,
    Joint,
    IKRootsHelper,
    setUrdfFromIK,
    urdfRobotToIKRoot,
    setIKFromUrdf,
    Goal,
    DOF
} from 'closed-chain-ik';

import URDFLoader from 'urdf-loader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree, MeshBVHVisualizer } from 'three-mesh-bvh';


Mesh.prototype.raycast = acceleratedRaycast;
BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;

const solverOptions = {
    maxIterations: 30,
    divergeThreshold: 0.01,
    stallThreshold: 0.00001,

    translationErrorClamp: 0.01,
    rotationErrorClamp: 0.01,

    translationConvergeThreshold: 1e-5,
    rotationConvergeThreshold: 1e-5,

    // Factors to apply to the translation and rotation error in the error vector.
    // Useful for weighting rotation higher than translation if they do not seem to
    // be solved "evenly". Values are expected to be in the range [ 0, 1 ].
    translationFactor: 1,
    rotationFactor: 1
};

let boxX = 1.1;
let boxY = 1;
let boxZ = 1.7;

const linkX = 2.5;
const linkY = 1.05;
const linkZ = 1.5;


let renderer, scene, camera;
let controls, transformControls, targetObject;
let directionalLight;

let urdfRoot, ikRoot, ikHelper, drawThroughIkHelper, solver;
let ikNeedsUpdate = true;

let R_followJoint, R_rukaLink;
let L_followJoint, L_rukaLink, L;
let head_followJoint, head_rukaLink;
let firstLayerId, secondLayerId;
let goal_ID = 1;
let goalMesh = [];
let followLink = [];
let followLinkMap = new Map()
let rArmPreset = [];
let lArmPreset = [];
let headPreset = [];

let geometryCube, materialCube, affectorR, affectorL, link4R, link4L, geometryLink;

let positionAffR = new Vector3;
let positionAffL = new Vector3;
let dirAffector = new Vector3;
let dirAffL = new Vector3;

let boundsViz, collHeadBig, collBodyBig;

let containerSendBtn = document.getElementById('sendBtn');
let sendBtn = document.createElement('BUTTON');
// sendBtn.id = 'sendId'
sendBtn.textContent = 'отправить';
containerSendBtn.appendChild(sendBtn);
// document.body.appendChild(sendBtn)

let mode1 = 'lol';
let mode2 = 'kek';

let containerModeBtn = document.getElementById('modeBtn');
let modeBtn = document.createElement('BUTTON');
modeBtn.textContent = mode1;
containerModeBtn.appendChild(modeBtn);
// document.body.appendChild(modeBtn)


let intersects



let body = {
    "l1": 0,
    "l2": 0,
    "l3": 0,
    "l4": 0,
    "l5": 0,
    "neck": 0,
    "head": 0,
    "r1": 0,
    "r2": 0,
    "r3": 0,
    "r4": 0,
    "r5": 0,
    "detach": true
}




// init renderer
renderer = new WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
let mainContainer = document.getElementById('mainWindow');
mainContainer.appendChild(renderer.domElement);
// document.body.appendChild(renderer.domElement);
camera = new PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 500);
camera.position.set(40, 30, 0);
scene = new Scene();
scene.background = new Color('grey');
//lights
scene.add(new HemisphereLight(0xffffff, 'grey', 2));
// controls
controls = new OrbitControls(camera, renderer.domElement);
controls.target.y = 15;
controls.update();

transformControls = new TransformControls(camera, renderer.domElement);
transformControls.setSize(0.7);
transformControls.setSpace('local');
scene.add(transformControls);

transformControls.addEventListener('mouseDown', () => {
    controls.enabled = false;
});

transformControls.addEventListener('mouseUp', () => {
    controls.enabled = true;
    ikNeedsUpdate = true;
    setIKFromUrdf(ikRoot, urdfRoot); // не дружит с transformControls
    ikRoot.updateMatrixWorld(true);
});
window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    const aspect = w / h;
    renderer.setSize(w, h);
    camera.aspect = aspect;
    camera.updateProjectionMatrix();
});

transformControls.addEventListener('objectChange', () => {
    ikNeedsUpdate = true;
});

// patrik  target
targetObject = new Group();
scene.add(targetObject);
transformControls.attach(targetObject);


const requestUrlGet = 'http://localhost:8000/api/current_position/'  
const requestUrlPost = 'http://localhost:8000/api/run_servo/'

// object
let multiplyerGet = 0.004712389
// let multiplyerGet = 212.206
// let offset = 2.356

let multiplyerPost = 212.20
let intervalSend = 100



sendRequest('GET', requestUrlGet)
    .then(
        data => {
 
            // rArmPreset[0] = data.positions.r1 / multiplyerGet - offset
            // rArmPreset[1] = data.positions.r2 / multiplyerGet - offset
            // rArmPreset[2] = data.positions.r3 / multiplyerGet - offset
            // rArmPreset[3] = data.positions.r4 / multiplyerGet - offset
            // rArmPreset[4] = data.positions.r5 / multiplyerGet - offset

            // lArmPreset[0] = data.positions.l1 / multiplyerGet - offset
            // lArmPreset[1] = data.positions.l2 / multiplyerGet - offset
            // lArmPreset[2] = data.positions.l3 / multiplyerGet - offset
            // lArmPreset[3] = data.positions.l4 / multiplyerGet - offset
            // lArmPreset[4] = data.positions.l5 / multiplyerGet - offset

            // headPreset[0] = data.positions.neck / multiplyerGet - offset
            // headPreset[1] = data.positions.head / multiplyerGet - offset

            rArmPreset[0] = data.positions.r1*  multiplyerGet
            rArmPreset[1] = data.positions.r2 * multiplyerGet
            rArmPreset[2] = data.positions.r3 * multiplyerGet
            rArmPreset[3] = data.positions.r4 * multiplyerGet
            rArmPreset[4] = data.positions.r5 * multiplyerGet

            lArmPreset[0] = data.positions.l1 * multiplyerGet
            lArmPreset[1] = data.positions.l2 * multiplyerGet
            lArmPreset[2] = data.positions.l3 * multiplyerGet
            lArmPreset[3] = data.positions.l4 * multiplyerGet
            lArmPreset[4] = data.positions.l5 * multiplyerGet

            headPreset[0] = data.positions.neck * multiplyerGet
            headPreset[1] = data.positions.head * multiplyerGet


            for (let key in data.positions) {

                //console.log(key);
                console.log(key + ' ' + data.positions[key]);

            }

            const loader = new URDFLoader();
            loader.parseCollision = true;

            loader
                .loadAsync('https://raw.githubusercontent.com/Gerka0291/urdf/main/bobotURDF/urdf/bobot.urdf')
                .then(
                    result => {
                        urdfRoot = result;

                        ikRoot = urdfRobotToIKRoot(urdfRoot)

                        urdfRoot.setJointValue('leftArm1_plast_link_joint', lArmPreset[0]);
                        urdfRoot.setJointValue('leftArm2_plast_link_joint', lArmPreset[1]);
                        urdfRoot.setJointValue('leftArm3_plast_link_joint', lArmPreset[2]);
                        urdfRoot.setJointValue('leftArm4_link_joint', lArmPreset[3]);
                        urdfRoot.setJointValue('leftArm5_plast_link_joint', lArmPreset[4]);

                        urdfRoot.setJointValue('leftArm1_link_joint', rArmPreset[0]);
                        urdfRoot.setJointValue('rightArm2_link_joint', rArmPreset[1]);
                        urdfRoot.setJointValue('rightArm3_link_joint', rArmPreset[2]);
                        urdfRoot.setJointValue('rightArm4_link_joint', lArmPreset[3]);
                        urdfRoot.setJointValue('rightArm5_plast_link_joint', lArmPreset[4]);

                        urdfRoot.setJointValue('neck_plast_link_joint', headPreset[0]);
                        urdfRoot.setJointValue('head_steel_link_joint', headPreset[1]);


                        // body.l1 = parseInt(urdfRoot.joints.leftArm1_plast_link_joint.angle * multiplyerPost)
                        // body.l2 = parseInt(urdfRoot.joints.leftArm2_plast_link_joint.angle * multiplyerPost)
                        // body.l3 = parseInt(urdfRoot.joints.leftArm3_plast_link_joint.angle * multiplyerPost)
                        // body.l4 = parseInt(urdfRoot.joints.leftArm4_link_joint.angle * multiplyerPost)
                        // body.l5 = parseInt(urdfRoot.joints.leftArm5_plast_link_joint.angle * multiplyerPost)

                        // body.neck = parseInt(urdfRoot.joints.leftArm1_plast_link_joint.angle * multiplyerPost)
                        // body.head = parseInt(urdfRoot.joints.head_steel_link_joint.angle * multiplyerPost)

                        // body.r1 = parseInt(urdfRoot.joints.leftArm1_link_joint.angle * multiplyerPost)
                        // body.r2 = parseInt(urdfRoot.joints.rightArm2_link_joint.angle * multiplyerPost)
                        // body.r3 = parseInt(urdfRoot.joints.rightArm3_link_joint.angle * multiplyerPost)
                        // body.r4 = parseInt(urdfRoot.joints.rightArm4_link_joint.angle * multiplyerPost)
                        // body.r5 = parseInt(urdfRoot.joints.rightArm5_plast_link_joint.angle * multiplyerPost)
                        setIKFromUrdf(ikRoot, urdfRoot);

                        // make base stationary
                        ikRoot.setDoF();

                        followLink[1] = new Link();
                        followLinkMap.set(1, new Link())
                        R_followJoint = new Joint();
                        R_followJoint.setDoF(DOF.EX, DOF.EY, DOF.EZ);
                        R_followJoint.setPosition(0, 0, 3);

                        R_rukaLink = ikRoot.find(l => l.name === 'rightArm5_red_link');
                        R_rukaLink.addChild(R_followJoint);
                        R_followJoint.addChild(followLinkMap.get(1));

                        // console.log(followLinkMap.get(1))
                        //followLink[2] = new Link();
                        followLinkMap.set(2, new Link())
                        L_followJoint = new Joint();
                        L_followJoint.setDoF(DOF.EX, DOF.EY, DOF.EZ);
                        L_followJoint.setPosition(0, 0, -3);

                        L_rukaLink = ikRoot.find(c => c.name === 'leftArm5_plast_link');
                        L_rukaLink.addChild(L_followJoint);
                        L_followJoint.addChild(followLinkMap.get(2));

                        //followLink[3] = new Link();
                        followLinkMap.set(3, new Link())
                        head_followJoint = new Joint();
                        head_followJoint.setDoF(DOF.EX, DOF.EY, DOF.EZ);
                        head_followJoint.setPosition(3, 5, 0);

                        head_rukaLink = ikRoot.find(c => c.name === 'head_steel_link');
                        head_rukaLink.addChild(head_followJoint);
                        head_followJoint.addChild(followLinkMap.get(3));

                        goalMesh[1] = new Goal();
                        goalMesh[2] = new Goal();
                        goalMesh[3] = new Goal();

                        // // followLinkMap.get(2).getWorldPosition(goalMesh[2].position)
                        // // followLinkMap.get(3).getWorldPosition(goalMesh[3].position)

                        goalMesh[1].makeClosure(followLinkMap.get(1));



                        solver = new Solver([ikRoot, goalMesh[goal_ID]]);


                        setIKFromUrdf(ikRoot, urdfRoot);
                        scene.add(urdfRoot);



                        setTimeout(() => {
                            collHeadBig = urdfRoot.colliders.head_coll_big.children[0];
                            collHeadBig.geometry.computeBoundsTree();


                            collBodyBig = urdfRoot.colliders.body0_coll_big.children[0];
                            collBodyBig.geometry.computeBoundsTree();



                            boundsViz = new MeshBVHVisualizer(collHeadBig);
                            boundsViz.depth = 100;

                            for (let key in urdfRoot.colliders) {
                                //	console.log(key);
                                urdfRoot.colliders[key].children[0].material.color.setHex(0xb8ac23);
                                urdfRoot.colliders[key].children[0].material.opacity = 0.3;
                                urdfRoot.colliders[key].children[0].material.transparent = true;
                                //console.log(urdfRoot.colliders[key]);
                            }

                        }, 500);
                        

                    }
                );

        })



    .catch(err => console.log(err))



let counter = 1
let txtmesaage =''
modeBtn.onclick = function () {
    txtmesaage =mode1
    counter += 1
    if (counter >= 2) {
        txtmesaage =mode2
        counter = 0
    }
    modeBtn.textContent = txtmesaage;
    console.log(counter)
}

setInterval(() => {
    if (counter === 0) {
        sendRequest('POST', requestUrlPost, body)
        //  .then(data => console.log(data))
        //  .catch(err => console.log(err))
        // console.log(body)

    }
}, intervalSend);

sendBtn.onclick = function () {
    // console.log('kek')
    sendRequest('POST', requestUrlPost, body)
        .then(data => console.log(data))
        .catch(err => console.log(err))

}


// --------------- make affectors ---------------------

geometryCube = new BoxGeometry(boxX, boxY, boxZ);
geometryLink = new BoxGeometry(linkX, linkY, linkZ);
materialCube = new MeshBasicMaterial({
    color: 0x049ef4,
    transparent: true,
    opacity: 0,

});
const materialSphere = new MeshBasicMaterial({
    color: 0x049ef4,
    transparent: true,
    opacity: 0.1,
    side: 1
});
affectorR = new Mesh(geometryCube, materialCube);
affectorR.matrixAutoUpdate = false;
affectorL = new Mesh(geometryCube, materialCube);
affectorL.matrixAutoUpdate = false;
link4R = new Mesh(geometryLink, materialCube);
link4R.matrixAutoUpdate = false;
link4L = new Mesh(geometryLink, materialCube);
link4L.matrixAutoUpdate = false;
scene.add(link4R);
scene.add(link4L);
scene.add(affectorR);
scene.add(affectorL);

const sphereGeometry = new SphereGeometry(1000)
const sphere = new Mesh(sphereGeometry, materialSphere)
scene.add(sphere)
console.log(materialCube)

//transformControls.attach(affectorR)



addEventListener("mousedown", function () {



    if ((firstLayerId || secondLayerId) == 'rightArm5_plast') {


        goal_ID = 1;
        goalMesh[goal_ID].makeClosure(followLinkMap.get(goal_ID));

        solver.updateStructure() //must to be, else dont work
        setIKFromUrdf(ikRoot, urdfRoot);

    } else if ((firstLayerId || secondLayerId) == 'head_coll_big') {

        goal_ID = 3;
        goalMesh[goal_ID].makeClosure(followLinkMap.get(goal_ID));

        solver.updateStructure() //must to be, else dont work
        setIKFromUrdf(ikRoot, urdfRoot);

    } else if ((firstLayerId || secondLayerId) == 'leftArm5_plast') {
        goal_ID = 2;
        goalMesh[goal_ID].makeClosure(followLinkMap.get(goal_ID));

        solver.updateStructure()//must to be, else dont work
        setIKFromUrdf(ikRoot, urdfRoot);

    }


});





//-----------------animate---------------------
function animate() {


    requestAnimationFrame(animate);

    // for (let key in urdfRoot.joints) {
    // sendArr.shift()
    //console.log(key);
    //console.log(key + ' ' + urdfRoot.joints[key].angle);
    // sendArr.push(urdfRoot.joints[key].angle)
    // }

    body.l1 = parseInt(urdfRoot.joints.leftArm1_plast_link_joint.angle * multiplyerPost)
    body.l2 = parseInt(urdfRoot.joints.leftArm2_plast_link_joint.angle * multiplyerPost)
    body.l3 = parseInt(urdfRoot.joints.leftArm3_plast_link_joint.angle * multiplyerPost)
    body.l4 = parseInt(urdfRoot.joints.leftArm4_link_joint.angle * multiplyerPost)
    body.l5 = parseInt(urdfRoot.joints.leftArm5_plast_link_joint.angle * multiplyerPost)

    body.neck = parseInt(urdfRoot.joints.neck_plast_link_joint.angle * multiplyerPost)
    body.head = parseInt(urdfRoot.joints.head_steel_link_joint.angle * multiplyerPost)

    body.r1 = parseInt(urdfRoot.joints.rightArm1_link_joint.angle * multiplyerPost)
    body.r2 = parseInt(urdfRoot.joints.rightArm2_link_joint.angle * multiplyerPost)
    body.r3 = parseInt(urdfRoot.joints.rightArm3_link_joint.angle * multiplyerPost)
    body.r4 = parseInt(urdfRoot.joints.rightArm4_link_joint.angle * multiplyerPost)
    body.r5 = parseInt(urdfRoot.joints.rightArm5_plast_link_joint.angle * multiplyerPost)
    //console.log(body)
    //console.log(' ')

    if (goal_ID == 1) {
        // right Arm constrains
        urdfRoot.visual.rightArm5_plast.children[0].getWorldPosition(positionAffR);
        targetObject.getWorldPosition(dirAffector);
        affectorR.position.copy(positionAffR);
        affectorR.translateZ(1.2);
        affectorR.lookAt(dirAffector);
        affectorR.updateMatrix();

        let distLink4r = dirAffector.distanceTo(positionAffR)

        link4R.position.copy(urdfRoot.visual.rightArm4.children[0].getWorldPosition(new Vector3))
        link4R.quaternion.copy(urdfRoot.visual.rightArm4.children[0].getWorldQuaternion(new Quaternion))
        link4R.translateZ(distLink4r / 3 - 1)
        link4R.updateMatrix()
        //console.log(distLink4);

        var transformMatrix = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(affectorR.matrixWorld);
        var transformMatrix1 = new Matrix4().copy(collBodyBig.matrixWorld).invert().multiply(affectorR.matrixWorld);
        var transformMatrix2 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(link4R.matrixWorld);
        var transformMatrix3 = new Matrix4().copy(collBodyBig.matrixWorld).invert().multiply(link4R.matrixWorld);
        const multiplyerBoxR = 1.6;
        const boxR = new Box3();
        boxR.min.set(-boxX / multiplyerBoxR, -boxY / multiplyerBoxR, -boxZ / multiplyerBoxR);
        boxR.max.set(boxX / multiplyerBoxR, boxY / multiplyerBoxR, boxZ / multiplyerBoxR);

        const multLinkR = 2;
        const linkBoxR = new Box3();
        linkBoxR.min.set(-linkX / multLinkR, -linkY / multLinkR, -linkZ / multLinkR)
        linkBoxR.max.set(linkX / multLinkR, linkY / multLinkR, linkZ / multLinkR)

        var hitheadR = collHeadBig.geometry.boundsTree.intersectsBox(boxR, transformMatrix)
        var hitBodyR = collBodyBig.geometry.boundsTree.intersectsBox(boxR, transformMatrix1)
        var hitheadsmallRlink = collHeadBig.geometry.boundsTree.intersectsBox(linkBoxR, transformMatrix2)
        var hitBodySmallRlink = collBodyBig.geometry.boundsTree.intersectsBox(linkBoxR, transformMatrix3)



    } else if (goal_ID == 2) {
        urdfRoot.visual.leftArm5_plast.children[0].getWorldPosition(positionAffL);
        targetObject.getWorldPosition(dirAffector);
        affectorL.position.copy(positionAffL);
        affectorL.translateZ(1.2);
        affectorL.lookAt(dirAffector);
        affectorL.updateMatrix();

        let distLink4l = dirAffector.distanceTo(positionAffL)

        link4L.position.copy(urdfRoot.visual.leftArm4.children[0].getWorldPosition(new Vector3))
        link4L.quaternion.copy(urdfRoot.visual.leftArm4.children[0].getWorldQuaternion(new Quaternion))
        link4L.translateZ(-distLink4l / 3 + 1)
        link4L.updateMatrix()
        //console.log(urdfRoot.visual);

        var transformMatrix5 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(affectorL.matrixWorld);
        var transformMatrix6 = new Matrix4().copy(collBodyBig.matrixWorld).invert().multiply(affectorL.matrixWorld);
        var transformMatrix7 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(link4L.matrixWorld);
        var transformMatrix8 = new Matrix4().copy(collBodyBig.matrixWorld).invert().multiply(link4L.matrixWorld);
        const multiplyerBoxR = 1.6;
        const boxL = new Box3();
        boxL.min.set(-boxX / multiplyerBoxR, -boxY / multiplyerBoxR, -boxZ / multiplyerBoxR);
        boxL.max.set(boxX / multiplyerBoxR, boxY / multiplyerBoxR, boxZ / multiplyerBoxR);

        const multLinkR = 2;
        const linkBoxL = new Box3();
        linkBoxL.min.set(-linkX / multLinkR, -linkY / multLinkR, -linkZ / multLinkR)
        linkBoxL.max.set(linkX / multLinkR, linkY / multLinkR, linkZ / multLinkR)

        var hitheadL = collHeadBig.geometry.boundsTree.intersectsBox(boxL, transformMatrix5)
        var hitBodyL = collBodyBig.geometry.boundsTree.intersectsBox(boxL, transformMatrix6)
        var hitheadsmallLlink = collHeadBig.geometry.boundsTree.intersectsBox(linkBoxL, transformMatrix7)
        var hitBodySmallLlink = collBodyBig.geometry.boundsTree.intersectsBox(linkBoxL, transformMatrix8)


    } else if (goal_ID == 3) {
        const multLinkR = 2;
        const multiplyerBoxR = 1.6;

        var transformMatrix = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(affectorR.matrixWorld);
        var transformMatrix2 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(link4R.matrixWorld);
        var transformMatrix5 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(affectorL.matrixWorld);
        var transformMatrix7 = new Matrix4().copy(collHeadBig.matrixWorld).invert().multiply(link4L.matrixWorld);

        const boxL = new Box3();
        boxL.min.set(-boxX / multiplyerBoxR, -boxY / multiplyerBoxR, -boxZ / multiplyerBoxR);
        boxL.max.set(boxX / multiplyerBoxR, boxY / multiplyerBoxR, boxZ / multiplyerBoxR);
        const linkBoxL = new Box3();
        linkBoxL.min.set(-linkX / multLinkR, -linkY / multLinkR, -linkZ / multLinkR)
        linkBoxL.max.set(linkX / multLinkR, linkY / multLinkR, linkZ / multLinkR)
        const boxR = new Box3();
        boxR.min.set(-boxX / multiplyerBoxR, -boxY / multiplyerBoxR, -boxZ / multiplyerBoxR);
        boxR.max.set(boxX / multiplyerBoxR, boxY / multiplyerBoxR, boxZ / multiplyerBoxR);
        const linkBoxR = new Box3();
        linkBoxR.min.set(-linkX / multLinkR, -linkY / multLinkR, -linkZ / multLinkR)
        linkBoxR.max.set(linkX / multLinkR, linkY / multLinkR, linkZ / multLinkR)

        var hitheadR = collHeadBig.geometry.boundsTree.intersectsBox(boxR, transformMatrix)
        var hitheadsmallRlink = collHeadBig.geometry.boundsTree.intersectsBox(linkBoxR, transformMatrix2)
        var hitheadL = collHeadBig.geometry.boundsTree.intersectsBox(boxL, transformMatrix5)
        var hitheadsmallLlink = collHeadBig.geometry.boundsTree.intersectsBox(linkBoxL, transformMatrix7)


    }

    if (hitheadR || hitBodyR || hitheadsmallRlink || hitBodySmallRlink || hitheadL || hitBodyL || hitheadsmallLlink || hitBodySmallLlink) {

    } else {

        solver.solve();
        // console.log('norm')
    }

    Object.assign(solver, solverOptions);
    setUrdfFromIK(urdfRoot, ikRoot);
    updateTarget();

    renderer.render(scene, camera);

    intersects = transformControls.getRaycaster().intersectObject(scene, true)
    if (intersects.length > 0) {

        firstLayerId = intersects[0].object.parent.name;
        secondLayerId = intersects[1].object.parent.name;
        // console.log(goal_ID, intersects[1].object.parent.name, intersects.length) //.parent.name
        // console.log(goal_ID, intersects[0].object.parent.name, intersects.length) //.parent.name
        // console.log(goal_ID, intersects[0])
        // console.log(' '
    }

}



animate();



function updateTarget() {

    if (!transformControls.dragging) {

        targetObject.matrix.set(...followLinkMap.get(goal_ID).matrixWorld).transpose();  //
        targetObject.matrix.decompose(
            targetObject.position,
            targetObject.quaternion,
            targetObject.scale,
        );

        goalMesh[goal_ID].setPosition(
            targetObject.position.x,
            targetObject.position.y,
            targetObject.position.z,
        );
        //console.log('rem')
    }
    goalMesh[goal_ID].setPosition(
        targetObject.position.x,
        targetObject.position.y,
        targetObject.position.z,
    );
}

function sendRequest(method, url, body = null) {
    return new Promise((resolve, reject) => {
        const xhr = new XMLHttpRequest()
        xhr.open(method, url)
        xhr.responseType = 'json'
        xhr.setRequestHeader('Content-Type', 'application/json')
        xhr.onload = () => {
            if (xhr.status >= 400) {
                reject(xhr.response)
            } else {
                resolve(xhr.response)
            }
        }
        xhr.onerror = () => {
            reject(xhr.response)
        }
        // 
        xhr.send(JSON.stringify(body))
    })

}