import {
	WebGLRenderer,
	PerspectiveCamera,
	Color,
	Scene,
	sRGBEncoding,
	Group,
	Raycaster,
	Vector2,
	Vector4,
	Mesh,
	SphereGeometry,
	MeshBasicMaterial,
	PCFSoftShadowMap,
	Box3,
	Sphere,
	Vector3,
	HemisphereLight,
	BufferGeometry,
	BoxGeometry,
	Matrix4,
	Quaternion



} from 'three';
import { OrbitControls, } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import Stats from 'three/examples/jsm/libs/stats.module.js';
import { mat4 } from 'gl-matrix';
import {
	Solver,
	WorkerSolver,
	Link,
	Joint,
	IKRootsHelper,
	setUrdfFromIK,

	urdfRobotToIKRoot,
	setIKFromUrdf,
	Goal,
	DOF
} from 'closed-chain-ik';
import { loadPatrik } from './loadPatrik.js'

import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree, MeshBVHVisualizer } from 'three-mesh-bvh';
Mesh.prototype.raycast = acceleratedRaycast;
BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
let collHeadBig, collBodyBig;
// let results

let geometryCube, materialCube, affectorR, affectorL, link4R, link4L, geometryLink;
let boxX = 1.1;
let boxY = 1;
let boxZ = 1.7;

const linkX = 2.5;
const linkY = 1.05;
const linkZ = 1.5;

let positionAffR = new Vector3;
let positionAffL = new Vector3;
let dirAffector = new Vector3;


const params = {
	solve: true,
	displayIk: false,
	displayGoals: true,

};

let solverOptions = {
	// useSVD: false,
	maxIterations: 30,
	divergeThreshold: 0.01,
	stallThreshold: 0.00001,

	translationErrorClamp: 0.01,
	rotationErrorClamp: 0.01,

	translationConvergeThreshold: 1e-5,
	rotationConvergeThreshold: 1e-5,

	translationFactor: 1,
	rotationFactor: 1,

	restPoseFactor: 0.025,
};


const goalToLinkMap = new Map();
const linkToGoalMap = new Map();
const goals = [];
const goalIcons = [];
let selectedGoalIndex = - 1;

let stats;
let outputContainer, renderer, scene, camera, mainContainer;
let solver, ikHelper, drawThroughIkHelper, ikRoot, urdfRoot;
let controls, transformControls, targetObject;
let mouse = new Vector2();
const box = new Box3();
const sphere = new Sphere();
const vector = new Vector3();


let rArmPreset = [];
let lArmPreset = [];
let headPreset = [];
let multiplyerGet = 212.206
let offset = 2.356
let offsetSend = 500

// const requestUrlPost = 'http://localhost:8000/api/run_servo/'
const requestUrlPost = 'http://192.168.1.43:8003/api/run_servo/'


let multiplyerPost = 212.20
let intervalSend = 700

let bodySend = {
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




// console.log('kek')

init();
loadModel(loadPatrik());


// --------------- работа с отправкой ------------------//
let mode1 = 'авто';
let mode2 = 'ручной';
let counter = 1
let txtmesaage = ''

let containerSendBtn = document.getElementById('sendBtn');
let sendBtn = document.createElement('BUTTON');
sendBtn.textContent = 'отправить';
containerSendBtn.appendChild(sendBtn);
// console.log(scene)
// raycast



let containerModeBtn = document.getElementById('modeBtn');
let modeBtn = document.createElement('BUTTON');
modeBtn.textContent = mode2;
containerModeBtn.appendChild(modeBtn);

modeBtn.onclick = function () {
	txtmesaage = mode2
	counter += 1
	if (counter >= 2) {
		txtmesaage = mode1
		counter = 0
	}
	modeBtn.textContent = txtmesaage;
	console.log(counter)
}

setInterval(() => {
	if (counter === 0) {
		sendRequest('POST', requestUrlPost, bodySend)
		//  .then(data => console.log(data))
		//  .catch(err => console.log(err))
		// console.log(bodySend)

	}
}, intervalSend);

sendBtn.onclick = function () {
	// console.log('kek')
	sendRequest('POST', requestUrlPost, bodySend)
		.then(data => console.log(data))
		.catch(err => console.log(err))
	console.log(bodySend)

}


render();


function init() {

	// stats = new Stats();
	// document.getElementById('mainWindow').appendChild(stats.dom);



	// init renderer
	renderer = new WebGLRenderer({ antialias: true });
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(window.innerWidth, window.innerHeight);
	mainContainer = document.getElementById('mainWindow');
	mainContainer.appendChild(renderer.domElement);

	camera = new PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 500);
	camera.position.set(40, 30, 0);
	// camera.rotation.set(1,1,1)
	// camera.rotateY(180);
	// camera.updateProjectionMatrix();
	console.log(camera)


	scene = new Scene();
	scene.background = new Color('#bed0e8');

	scene.add(new HemisphereLight(0xffffff, 'grey', 2));
	controls = new OrbitControls(camera, renderer.domElement);
	controls.target.y = 15;
	controls.update();



	transformControls = new TransformControls(camera, renderer.domElement);
	transformControls.setSpace('world'); // local
	transformControls.setSize(0.7);
	scene.add(transformControls);

	transformControls.addEventListener('mouseDown', () => controls.enabled = false);
	transformControls.addEventListener('mouseUp', () => controls.enabled = true);

	targetObject = new Group();
	targetObject.position.set(0, 1, 1);
	scene.add(targetObject);
	transformControls.attach(targetObject);

	// --------------- создание кнопок---------------------//


	window.addEventListener('resize', () => {

		const w = window.innerWidth;
		const h = window.innerHeight;
		const aspect = w / h;
		renderer.setSize(w, h);
		camera.aspect = aspect;
		camera.updateProjectionMatrix();

		if (ikHelper) {

			ikHelper.setResolution(window.innerWidth, window.innerHeight);
			drawThroughIkHelper.setResolution(window.innerWidth, window.innerHeight);

		}

	});

	window.addEventListener('keydown', e => {

		switch (e.key) {

			case 'w':
				transformControls.setMode('translate');
				break;
			case 'e':
				transformControls.setMode('rotate');
				break;
			case 'q':
				transformControls.setSpace(transformControls.space === 'local' ? 'world' : 'local');
				break;
			case 'f':
				controls.target.set(0, 0, 0);
				controls.update();
				break;

		}

	});

	transformControls.addEventListener('mouseUp', () => {

		if (selectedGoalIndex !== - 1) {

			const goal = goals[selectedGoalIndex];
			const ikLink = goalToLinkMap.get(goal);
			if (ikLink) {

				ikLink.updateMatrixWorld();

				ikLink.attachChild(goal);
				goal.setPosition(...goal.originalPosition);
				goal.setQuaternion(...goal.originalQuaternion);
				ikLink.detachChild(goal);

				targetObject.position.set(...goal.position);
				targetObject.quaternion.set(...goal.quaternion);

			}

		}

	});

	renderer.domElement.addEventListener('pointerdown', e => {

		mouse.x = e.clientX;
		mouse.y = e.clientY;

	});

	renderer.domElement.addEventListener('pointerup', e => {

		if (Math.abs(e.clientX - mouse.x) > 3 || Math.abs(e.clientY - mouse.y) > 3) return;

		if (!urdfRoot) return;

		const { ikLink, result } = raycast(e);

		// console.log( ikLink, result )

		if (ikLink === null) {

			selectedGoalIndex = - 1;

		}
		// console.log(e.button)
		if (e.button === 0) {


			if (!transformControls.dragging) {

				selectedGoalIndex = goalIcons.indexOf(result ? result.object.parent : null);

				if (selectedGoalIndex !== - 1) {

					const ikgoal = goals[selectedGoalIndex];
					targetObject.position.set(...ikgoal.position);
					targetObject.quaternion.set(...ikgoal.quaternion);

				} else if (ikLink && linkToGoalMap.has(ikLink)) {

					const goal = linkToGoalMap.get(ikLink);
					selectedGoalIndex = goals.indexOf(goal);
					targetObject.position.set(...goal.position);
					targetObject.quaternion.set(...goal.quaternion);

				}

			}

		}

	});

	window.addEventListener('keydown', e => {

		if (selectedGoalIndex !== - 1 && (e.code === 'Delete' || e.code === 'Backspace')) {

			deleteGoal(goals[selectedGoalIndex]);
			selectedGoalIndex = - 1;

		}

	});





	function deleteGoal(goal) {

		const index = goals.indexOf(goal);
		const goalToRemove = goals[index];
		goalToRemove.traverse(c => {

			if (c.isClosure) {

				c.removeChild(c.child);

			}

		});

		goals.splice(index, 1);

		const link = goalToLinkMap.get(goalToRemove);
		goalToLinkMap.delete(goalToRemove);
		linkToGoalMap.delete(link);

		solver.updateStructure();
		ikHelper.updateStructure();
		drawThroughIkHelper.updateStructure();

	}




	// --------------- make affectors ---------------------

	geometryCube = new BoxGeometry(boxX, boxY, boxZ);
	geometryLink = new BoxGeometry(linkX, linkY, linkZ);
	materialCube = new MeshBasicMaterial({
		color: 0x049ef4,
		transparent: true,
		opacity: 0.0,

	});
	const materialSphere = new MeshBasicMaterial({
		color: 0x049ef4,
		transparent: true,
		opacity: 0.3,
		// side: 1
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
}

function raycast(e) {

	let results;
	let intersectGoals = [...goalIcons];
	intersectGoals.length = goals.length;
	results = transformControls.getRaycaster().intersectObjects(intersectGoals, true)

	if (results.length !== 0) {
		return { ikLink: null, result: results[0] };
	}

	if (results.length === 0) {
		return { ikLink: null, result: null };
	}

	const result = results[0];

	let nearestLink = null;
	let ikLink = null;
	result.object.traverseAncestors(p => {

		if (nearestLink === null && p.isURDFLink) {
			nearestLink = p;
			ikRoot.traverse(c => {
				if (c.name === nearestLink.name) {
					ikLink = c;
				}
			});
		}
	});
	return { ikLink, result };
}

function render() {

	requestAnimationFrame(render);

	bodySend.r5 = scale(parseInt(urdfRoot.joints.leftArm1_plast_joint.angle * multiplyerPost + offsetSend), 0, 1000, 1000, 0)
	bodySend.r4 = parseInt(urdfRoot.joints.leftArm2_plast_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.r3 = parseInt(urdfRoot.joints.leftArm3_plast_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.r2 = parseInt(urdfRoot.joints.leftArm4_Joint.angle * multiplyerPost + offsetSend)
	bodySend.r1 = scale(parseInt(urdfRoot.joints.leftArm5_plast_link_joint.angle * multiplyerPost + offsetSend), 0, 1000, 1000, 0)

	bodySend.head = parseInt(urdfRoot.joints.neck_plast_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.neck = scale(parseInt(urdfRoot.joints.head_steel_Joint.angle * multiplyerPost + offsetSend), 0, 1000, 1000, 0)

	bodySend.l5 = parseInt(urdfRoot.joints.rightArm1_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.l4 = parseInt(urdfRoot.joints.rightArm2_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.l3 = parseInt(urdfRoot.joints.rightArm3_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.l2 = parseInt(urdfRoot.joints.rightArm4_link_joint.angle * multiplyerPost + offsetSend)
	bodySend.l1 = parseInt(urdfRoot.joints.rightArm5_plast_link_joint.angle * multiplyerPost + offsetSend)
	// console.log(bodySend)

	collHeadBig = urdfRoot.colliders.head_coll_big.children[0];
	collHeadBig.geometry.computeBoundsTree();
	collBodyBig = urdfRoot.colliders.body0_coll_big.children[0];
	collBodyBig.geometry.computeBoundsTree();

	// console.log(collHeadBig)


	// console.log(selectedGoalIndex)
	const allGoals = goals;
	const selectedGoal = allGoals[selectedGoalIndex];
	if (ikRoot) {



		if (selectedGoal) {

			selectedGoal.setPosition(targetObject.position.x, targetObject.position.y, targetObject.position.z);
			selectedGoal.setQuaternion(targetObject.quaternion.x, targetObject.quaternion.y, targetObject.quaternion.z, targetObject.quaternion.w);

		}


		if (selectedGoalIndex == 2) {    //(goal_ID == 1) 
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



		} else if (selectedGoalIndex == 1) {
			//lArm constrains (goal_ID == 2)
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


		} else if (selectedGoalIndex == 0) {
			// head constrains(goal_ID == 3)
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

			console.log('NEnorm')
		}
		else {

			solver.solve();
			// console.log('norm')
			const startTime = window.performance.now();
			let statuses;

			Object.assign(solver, solverOptions);
			statuses = solver.solve();

			setUrdfFromIK(urdfRoot, ikRoot);
		}

	}

	while (goalIcons.length < allGoals.length) {

		const color = new Color(0xffca28).convertSRGBToLinear();
		const group = new Group();
		const mesh = new Mesh(
			new SphereGeometry(0.05, 30, 30),
			new MeshBasicMaterial({ color }),
		);
		const mesh2 = new Mesh(
			new SphereGeometry(0.05, 30, 30),
			new MeshBasicMaterial({
				color,
				opacity: 0.4,
				transparent: true,
				depthWrite: false,
				depthTest: false,
			}),
		);
		// consistent size in screen space.

		const ogUpdateMatrix = mesh.updateMatrix;
		function updateMatrix(...args) {
			this.scale.setScalar(this.position.distanceTo(camera.position) * 0.15);
			ogUpdateMatrix.call(this, ...args);
		}
		mesh.updateMatrix = updateMatrix;
		mesh2.updateMatrix = updateMatrix;
		group.add(mesh, mesh2);
		scene.add(group);
		goalIcons.push(group);

	}

	goalIcons.forEach(g => g.visible = false);
	allGoals.forEach((g, i) => {

		goalIcons[i].position.set(...g.position);
		goalIcons[i].quaternion.set(...g.quaternion);
		goalIcons[i].visible = params.displayGoals;

	});

	// transformControls.enabled = selectedGoalIndex !== - 1;
	transformControls.visible = selectedGoalIndex !== - 1;

	setColor()
	renderer.render(scene, camera);
	// stats.update();

}





function loadModel(promise) {


	promise
		.then(({ goalMap, urdf, ik, helperScale = 6 }) => {
			ik.updateMatrixWorld(true);

			// create the helper
			if (params.displayIk) {
				ikHelper = new IKRootsHelper(ik);
				ikHelper.setJointScale(helperScale);
				ikHelper.setResolution(window.innerWidth, window.innerHeight);
				ikHelper.color.set(0xe91e63).convertSRGBToLinear();
				ikHelper.setColor(ikHelper.color);

				drawThroughIkHelper = new IKRootsHelper(ik);
				drawThroughIkHelper.setJointScale(helperScale);
				drawThroughIkHelper.setResolution(window.innerWidth, window.innerHeight);
				drawThroughIkHelper.color.set(0xe91e63).convertSRGBToLinear();
				drawThroughIkHelper.setColor(drawThroughIkHelper.color);
				drawThroughIkHelper.setDrawThrough(true);
				scene.add(ikHelper, drawThroughIkHelper);
			}
			// console.log(urdf.colliders.body0_coll_big)

			scene.add(urdf);

			const loadedGoals = [];
			goalMap.forEach((link, goal) => {

				loadedGoals.push(goal);
				goalToLinkMap.set(goal, link);
				linkToGoalMap.set(link, goal);
			});

			solver = new Solver(ik);


			if (loadedGoals.length) {

				targetObject.position.set(...loadedGoals[0].position);
				targetObject.quaternion.set(...loadedGoals[0].quaternion);
				selectedGoalIndex = 0;
			} else {
				selectedGoalIndex = - 1;
			}
			loadedGoals.forEach(g => {

				g.originalPosition = [0, 0, 0];
				g.originalQuaternion = [0, 0, 0, 1];
			});

			ikRoot = ik;
			urdfRoot = urdf;
			// console.log(urdfRoot.visual.body0_plast_white.children)
			goals.push(...loadedGoals);




		});



}
function setColor() {
	for (let key in urdfRoot.colliders) {
		//	console.log(key);
		urdfRoot.colliders[key].children[0].material.color.setHex(0xb8ac23);
		urdfRoot.colliders[key].children[0].material.opacity = 0.5;
		urdfRoot.colliders[key].children[0].material.transparent = true;
		//console.log(urdfRoot.colliders[key]);
	}
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

function scale(number, inMin, inMax, outMin, outMax) {
	return (number - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}