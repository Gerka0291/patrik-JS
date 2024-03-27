
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
import { LoadingManager } from 'three';
import { quat } from 'gl-matrix';

let Larm = 0.2
let Rarm = -0.2

let Head = 0


let ik, urdf, goalMap;


// const requestUrlGet = 'http://localhost:8000/api/current_position/'
const requestUrlGet = 'http://192.168.1.43:8003/api/current_position/'


let rArmPreset = [];
let lArmPreset = [];
let headPreset = [];
let multiplyerGet = 212.206
let offset = 2.356
let offsetSend = 500

let multiplyerPost = 212.20
let intervalSend = 100



export function loadPatrik() {





    // console.log("hello parcel");

    return new Promise((resolve, reject) => {


        sendRequest('GET', requestUrlGet)
            .then(
                data => {



                    lArmPreset[4] = (data.positions.r1 / multiplyerGet - offset)*-1
                    lArmPreset[3] = data.positions.r2 / multiplyerGet - offset
                    lArmPreset[2] = data.positions.r3 / multiplyerGet - offset
                    lArmPreset[1] = data.positions.r4 / multiplyerGet - offset
                    lArmPreset[0] = (data.positions.r5 / multiplyerGet - offset)*-1

                    rArmPreset[4] = data.positions.l1 / multiplyerGet - offset
                    rArmPreset[3] = data.positions.l2 / multiplyerGet - offset
                    rArmPreset[2] = data.positions.l3 / multiplyerGet - offset
                    rArmPreset[1] = data.positions.l4 / multiplyerGet - offset
                    rArmPreset[0] = data.positions.l5 / multiplyerGet - offset

                    headPreset[1] = (data.positions.neck / multiplyerGet - offset) *-1
                    headPreset[0] = (data.positions.head / multiplyerGet - offset)

                    for (let key in data.positions) {

                        //console.log(key);
                        console.log(key + ' ' + data.positions[key]);

                    }
                    for (var item in lArmPreset) {
                        console.log(lArmPreset[item])
                    }


                    const manager = new LoadingManager();
                    manager.onLoad = () => {

                        resolve({ ik, urdf, goalMap });

                    };

                    const url = 'https://raw.githubusercontent.com/Gerka0291/urdf/main/bobotURDF/urdf/bobot.urdf';

                    const loader = new URDFLoader(manager);
                    loader.parseCollision = true;
                    loader.load(url, result => {

                        urdf = result;
                        ik = urdfRobotToIKRoot(urdf);

                        // make the root fixed
                        ik.clearDoF();
                        // ik.setMatrixNeedsUpdate();

                        // update the robot joints
                        // console.log(urdf.joints);
                        urdf.joints['leftArm1_plast_joint'].setJointValue(lArmPreset[0]);
                        urdf.joints['leftArm2_plast_link_joint'].setJointValue(lArmPreset[1]);
                        urdf.joints['leftArm3_plast_link_joint'].setJointValue(lArmPreset[2]);
                        urdf.joints['leftArm4_Joint'].setJointValue(lArmPreset[3]);
                        urdf.joints['leftArm5_plast_link_joint'].setJointValue(lArmPreset[4]);

                        urdf.joints['rightArm1_link_joint'].setJointValue(rArmPreset[0]);
                        urdf.joints['rightArm2_link_joint'].setJointValue(rArmPreset[1]);
                        urdf.joints['rightArm3_link_joint'].setJointValue(rArmPreset[2]);
                        urdf.joints['rightArm4_link_joint'].setJointValue(rArmPreset[3]);
                        urdf.joints['rightArm5_plast_link_joint'].setJointValue(rArmPreset[4]);

                        urdf.joints['neck_plast_link_joint'].setJointValue(headPreset[0]);
                        urdf.joints['head_steel_Joint'].setJointValue(headPreset[1]);

                        // urdf.joints['leftArm1_plast_joint'].setJointValue(Larm);
                        // urdf.joints['leftArm2_plast_link_joint'].setJointValue(Larm);
                        // urdf.joints['leftArm3_plast_link_joint'].setJointValue(Larm);
                        // urdf.joints['leftArm4_Joint'].setJointValue(Larm);
                        // urdf.joints['leftArm5_plast_link_joint'].setJointValue(Larm);

                        // urdf.joints['rightArm1_link_joint'].setJointValue(Larm);
                        // urdf.joints['rightArm2_link_joint'].setJointValue(Larm);
                        // urdf.joints['rightArm3_link_joint'].setJointValue(Larm);
                        // urdf.joints['rightArm4_link_joint'].setJointValue(Larm);
                        // urdf.joints['rightArm5_plast_link_joint'].setJointValue(Larm);

                        // urdf.joints['neck_plast_link_joint'].setJointValue(Larm);
                        // urdf.joints['head_steel_Joint'].setJointValue(Larm);


                        // urdf.joints["rArmEff_joint"].jointType = 'floating'
                        // urdf.joints["lArmEff_joint"].jointType = 'floating'



                        setIKFromUrdf(ik, urdf);
                        urdf.colliders.body0_coll_big.visible = false
                        urdf.colliders.head_coll_big.visible = false



                        goalMap = new Map();

                        ik.traverse(c => {

                            if (c.isJoint) {

                                c.dofRestPose.set(c.dofValues);
                                c.restPoseSet = true;
                                // console.log(c)

                            } else if (

                                c.name === 'HeadEff_link' ||
                                c.name === 'lArmEff_link' ||   //leftArm5_plast_link L_followLink
                                c.name === "rArmEff_link"

                            ) {
                                // console.log(c)

                                urdf.links[c.name].visible=false
                                const link = urdf.links[c.name];
                                const ee = new Joint();
                                ee.setDoF(DOF.EX, DOF.EY, DOF.EZ)
                                ee.name = link.name;
                                ee.makeClosure(c);

                                c.getWorldPosition(ee.position);
                                c.getWorldQuaternion(ee.quaternion);
                                ee.setMatrixNeedsUpdate();

                                goalMap.set(ee, c);
                                // console.log(goalMap)

                            }

                        });

                    }, null, reject);

                });
    });
}

// loadPatrik()


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