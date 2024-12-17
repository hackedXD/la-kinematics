import {
	Environment,
	Grid,
	Html,
	OrbitControls,
	TransformControls,
} from "@react-three/drei";
import { Canvas, useLoader, useThree } from "@react-three/fiber";
import { button, useControls } from "leva";
import { useEffect, useRef, useState } from "react";
import {
	Color,
	Matrix3,
	Matrix4,
	Mesh,
	MeshStandardMaterial,
	SphereGeometry,
	Vector3,
	Vector4,
} from "three";
import "katex/dist/katex.min.css";
import URDFLoader, { URDFJoint, URDFRobot } from "urdf-loader";
import Latex from "react-latex";

function debounce(func, timeout = 300) {
	let timer: number;
	return (...args) => {
		clearTimeout(timer);
		timer = setTimeout(() => {
			func.apply(this, args);
		}, timeout);
	};
}

function forwardKinematics(t1: number, t2: number, t3: number, t4: number) {
	const px =
		-(
			0.317 * Math.sin(t2) +
			0.202 * Math.sin(t2 + t3) +
			0.1605 * Math.sin(t2 + t3 + t4)
		) * Math.sin(t1);
	const py =
		0.317 * Math.cos(t2) +
		0.202 * Math.cos(t2 + t3) +
		0.1605 * Math.cos(t2 + t3 + t4) +
		0.1775;
	const pz =
		-(
			0.317 * Math.sin(t2) +
			0.202 * Math.sin(t2 + t3) +
			0.1605 * Math.sin(t2 + t3 + t4)
		) * Math.cos(t1);

	let position = new Vector3(px, py, pz);
	position = position.multiplyScalar(4);
	position.applyAxisAngle(new Vector3(0, 1, 0), -Math.PI / 2);

	return position;
}

function computeJacobian(t1, t2, t3, t4, delta = 1e-6) {
	const currentPos = forwardKinematics(t1, t2, t3, t4);
	const J = [];

	// Perturb each joint angle slightly and compute partial derivatives
	const angles = [t1, t2, t3, t4];
	for (let i = 0; i < 4; i++) {
		const perturbedAngles: number[] = [...angles];
		perturbedAngles[i] += delta;

		const newPos = forwardKinematics(...perturbedAngles);
		const column = new Vector3()
			.subVectors(newPos, currentPos)
			.multiplyScalar(1 / delta);

		J.push([column.x, column.y, column.z]);
	}

	// Transpose Jacobian (3x4)
	return [
		[J[0][0], J[1][0], J[2][0], J[3][0]],
		[J[0][1], J[1][1], J[2][1], J[3][1]],
		[J[0][2], J[1][2], J[2][2], J[3][2]],
	];
}

function pseudoInverse(J, lambda = 1e-4) {
	// Convert Jacobian (3x4) to THREE.Matrix4 for manipulations
	const JT = new Matrix4();
	JT.set(
		J[0][0],
		J[1][0],
		J[2][0],
		0,
		J[0][1],
		J[1][1],
		J[2][1],
		0,
		J[0][2],
		J[1][2],
		J[2][2],
		0,
		J[0][3],
		J[1][3],
		J[2][3],
		0,
	);
	const J_transpose = JT.clone().transpose();

	// Compute J * JT (3x3 matrix)
	const JJT = new Matrix3();
	JJT.set(
		J[0][0] ** 2 + J[1][0] ** 2 + J[2][0] ** 2,
		J[0][0] * J[0][1] + J[1][0] * J[1][1] + J[2][0] * J[2][1],
		J[0][0] * J[0][2] + J[1][0] * J[1][2] + J[2][0] * J[2][2],

		J[0][1] * J[0][0] + J[1][1] * J[1][0] + J[2][1] * J[2][0],
		J[0][1] ** 2 + J[1][1] ** 2 + J[2][1] ** 2,
		J[0][1] * J[0][2] + J[1][1] * J[1][2] + J[2][1] * J[2][2],

		J[0][2] * J[0][0] + J[1][2] * J[1][0] + J[2][2] * J[2][0],
		J[0][2] * J[0][1] + J[1][2] * J[1][1] + J[2][2] * J[2][1],
		J[0][2] ** 2 + J[1][2] ** 2 + J[2][2] ** 2,
	);

	// Add damping term λ² * I to JJT to make it invertible
	JJT.elements[0] += lambda ** 2;
	JJT.elements[4] += lambda ** 2;
	JJT.elements[8] += lambda ** 2;

	// Invert the regularized JJT matrix
	const JJT_inv = JJT.clone().invert();

	// Compute the pseudo-inverse: J^+ = JT * (JJT)^-1
	const pseudoInverse = new Matrix4();
	pseudoInverse.multiplyMatrices(
		J_transpose,
		new Matrix4().setFromMatrix3(JJT_inv),
	);

	return pseudoInverse;
}

// Main IK Solver
function inverseKinematics(
	target: Vector3,
	initialAngles: number[],
	angleLimits: [number, number][],
	tolerance = 1e-4,
	maxIterations = 100,
) {
	let angles = [...initialAngles];
	let errorNorm = Infinity;

	for (let iter = 0; iter < maxIterations; iter++) {
		const currentPos = forwardKinematics(...angles);
		const error = new Vector3().subVectors(target, currentPos);

		// Check for convergence
		errorNorm = error.length();
		if (errorNorm < tolerance) {
			console.log(`Converged in ${iter} iterations`);
			break;
		}

		// Compute Jacobian and its pseudo-inverse
		const J = computeJacobian(...angles);
		const J_pinvMatrix = pseudoInverse(J);
		const J_pinv = [
			J_pinvMatrix.elements[0],
			J_pinvMatrix.elements[1],
			J_pinvMatrix.elements[2],
			J_pinvMatrix.elements[4],
			J_pinvMatrix.elements[5],
			J_pinvMatrix.elements[6],
			J_pinvMatrix.elements[8],
			J_pinvMatrix.elements[9],
			J_pinvMatrix.elements[10],
			J_pinvMatrix.elements[12],
			J_pinvMatrix.elements[13],
			J_pinvMatrix.elements[14],
		];

		// Compute deltaAngles: J_pinv * error
		const deltaAngles = [
			J_pinv[0] * error.x + J_pinv[1] * error.y + J_pinv[2] * error.z,
			J_pinv[3] * error.x + J_pinv[4] * error.y + J_pinv[5] * error.z,
			J_pinv[6] * error.x + J_pinv[7] * error.y + J_pinv[8] * error.z,
			J_pinv[9] * error.x + J_pinv[10] * error.y + J_pinv[11] * error.z,
		];

		// Apply a learning rate to stabilize updates
		const learningRate = 0.1;
		for (let i = 0; i < angles.length; i++) {
			angles[i] += learningRate * deltaAngles[i];
			// Normalize angles to [-π, π] to reduce sin error
			angles[i] = ((angles[i] + Math.PI) % (2 * Math.PI)) - Math.PI;
			// Clamp angles to respect angle limits
			angles[i] = Math.max(
				angleLimits[i][0],
				Math.min(angles[i], angleLimits[i][1]),
			);
		}

		console.log(`Iteration ${iter}, Error Norm: ${errorNorm.toFixed(6)}`);
	}

	return angles;
}

function URDFModel({ orbitRef }) {
	const robot: URDFRobot = useLoader(URDFLoader, "robot.urdf");
	const [joints, setJoints] = useState(robot.joints);
	const targetRef = useRef(null);
	const scene = useThree((state) => state.scene);
	const prevPosition = useRef(new Vector3());
	const onTargetMove = debounce(() => {
		if (!targetRef.current) return;

		const targetPosition = new Vector3();
		targetRef.current.getWorldPosition(targetPosition);

		const initialAngles = [
			"arm_joint_1",
			"arm_joint_2",
			"arm_joint_3",
			"arm_joint_4",
		].map((name) => joints[name].jointValue[0] as number);
		const angleLimits = [
			"arm_joint_1",
			"arm_joint_2",
			"arm_joint_3",
			"arm_joint_4",
		].map((name) => [
			joints[name].limit.lower as number,
			joints[name].limit.upper as number,
		]);

		const angles = inverseKinematics(
			targetPosition,
			initialAngles,
			angleLimits,
		);

		joints["arm_joint_1"].setJointValue(angles[0]);
		joints["arm_joint_2"].setJointValue(angles[1]);
		joints["arm_joint_3"].setJointValue(angles[2]);
		joints["arm_joint_4"].setJointValue(angles[3]);

		setJoints({ ...robot.joints });
	}, 20);

	const _ = useControls({
		runIK: button(() => {
			if (!targetRef.current) return;

			const targetPosition = new Vector3();
			targetRef.current.getWorldPosition(targetPosition);

			const initialAngles = [
				"arm_joint_1",
				"arm_joint_2",
				"arm_joint_3",
				"arm_joint_4",
			].map((name) => joints[name].jointValue[0] as number);
			const angleLimits = [
				"arm_joint_1",
				"arm_joint_2",
				"arm_joint_3",
				"arm_joint_4",
			].map((name) => [
				joints[name].limit.lower as number,
				joints[name].limit.upper as number,
			]);

			const angles = inverseKinematics(
				targetPosition,
				initialAngles,
				angleLimits,
			);

			let sphere = new Mesh(
				new SphereGeometry(0.1, 16, 16),
				new MeshStandardMaterial({ color: "red" }),
			);

			sphere.position.copy(forwardKinematics(...angles));

			joints["arm_joint_1"].setJointValue(angles[0]);
			joints["arm_joint_2"].setJointValue(angles[1]);
			joints["arm_joint_3"].setJointValue(angles[2]);
			joints["arm_joint_4"].setJointValue(angles[3]);

			console.log(angles);

			scene.add(sphere);
		}),
		...Object.keys(robot.joints).reduce(
			(acc, key) => {
				if (robot.joints[key].jointType === "fixed") return acc;

				acc[key] = {
					value: robot.joints[key].jointValue[0] as number,
					min: robot.joints[key].limit.lower as number,
					max: robot.joints[key].limit.upper as number,
					onChange: (value: number) => {
						robot.joints[key].setJointValue(value);
						setJoints({ ...robot.joints });
					},
				};

				return acc;
			},
			{} as Record<string, any>,
		),
	});

	useEffect(() => {
		robot.scale.set(4, 4, 4);
		robot.traverse((child) => {
			if (child instanceof Mesh) {
				child.material = new MeshStandardMaterial({
					color: child.material.color || new Color("white"), // Retain original color if present
					metalness: 0.15, // Add slight metallic effect
					roughness: 0.1, // Adjust roughness for balanced specularity
				});

				child.castShadow = true;
				child.receiveShadow = true;
			}
		});
	}, [robot]);

	return (
		<>
			<primitive object={robot} rotation={[-Math.PI / 2, 0, 0]} />
			{/* 
			X:  0.0545*cos(t1) + 0.317*cos(t3)*cos(t1 + t2) + 0.3625*cos(t1 + t2)*cos(t3 + t4) + 0.123*cos(t1 + t2)
			Y:  0.0545*sin(t1) + 0.317*sin(t1 + t2)*cos(t3) + 0.3625*sin(t1 + t2)*cos(t3 + t4) + 0.123*sin(t1 + t2)
			Z:  0.317*sin(t3) + 0.3625*sin(t3 + t4)
			*/}
			<mesh
				position={forwardKinematics(
					joints["arm_joint_1"].jointValue[0] as number,
					joints["arm_joint_2"].jointValue[0] as number,
					joints["arm_joint_3"].jointValue[0] as number,
					joints["arm_joint_4"].jointValue[0] as number,
				)}
				// position={[0, 1, 0]}
			>
				<sphereGeometry args={[0.1, 16, 16]} />
				<meshBasicMaterial color="green" />
			</mesh>
			{/* <mesh
				position={[
					-0.202 *
						Math.sin(joints["arm_joint_3"].jointValue[0]) *
						Math.cos(
							joints["arm_joint_1"].jointValue[0] +
								joints["arm_joint_2"].jointValue[0],
						) -
						0.1605 *
							Math.sin(joints["arm_joint_4"].jointValue[0]) *
							Math.cos(joints["arm_joint_3"].jointValue[0]) *
							Math.cos(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							) +
						0.1605 *
							Math.sin(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							) *
							Math.cos(joints["arm_joint_4"].jointValue[0]) -
						0.317 *
							Math.sin(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							),
					-0.202 *
						Math.sin(joints["arm_joint_3"].jointValue[0]) *
						Math.sin(
							joints["arm_joint_1"].jointValue[0] +
								joints["arm_joint_2"].jointValue[0],
						) -
						0.1605 *
							Math.sin(joints["arm_joint_4"].jointValue[0]) *
							Math.sin(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							) *
							Math.cos(joints["arm_joint_3"].jointValue[0]) -
						0.1605 *
							Math.cos(joints["arm_joint_4"].jointValue[0]) *
							Math.cos(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							) +
						0.317 *
							Math.cos(
								joints["arm_joint_1"].jointValue[0] +
									joints["arm_joint_2"].jointValue[0],
							),
					0.1605 *
						Math.sin(joints["arm_joint_3"].jointValue[0]) *
						Math.sin(joints["arm_joint_4"].jointValue[0]) -
						0.202 * Math.cos(joints["arm_joint_3"].jointValue[0]) +
						0.1775,
				]}
			>
				<sphereGeometry args={[0.1, 16, 16]} />
				<meshBasicMaterial color="red" />
			</mesh> */}

			<TransformControls
				enabled={true}
				showX
				showY
				showZ
				onMouseDown={() => (orbitRef.current!.enabled = false)}
				onPointerLeave={() => (orbitRef.current!.enabled = true)}
				onChange={(e) => {
					if (!targetRef.current) return;

					const targetPosition = new Vector3();
					targetRef.current.getWorldPosition(targetPosition);

					if (
						prevPosition.current &&
						prevPosition.current.equals(targetPosition)
					)
						return;
					prevPosition.current = targetPosition;

					onTargetMove();
				}}
			>
				<mesh castShadow ref={targetRef}>
					<sphereGeometry args={[0.1, 16, 16]} />
					<meshStandardMaterial color="pink" />
				</mesh>
			</TransformControls>

			{Object.keys(joints).map((name) => (
				<JointMarker key={name} name={name} joints={robot.joints} />
			))}
		</>
	);
}

function JointMarker({
	name,
	joints,
}: {
	name: string;
	joints: Record<string, URDFJoint>;
}) {
	const [hovered, setHovered] = useState(false);
	const position = new Vector3();
	joints[name].getWorldPosition(position);

	return (
		<>
			<mesh position={position}>
				<sphereGeometry args={[0.05, 16, 16]} />
				<meshStandardMaterial color="blue" />
			</mesh>
			<mesh
				position={position}
				onPointerOver={() => setHovered(true)}
				onPointerOut={() => setHovered(false)}
			>
				<sphereGeometry args={[0.15, 16, 16]} />
				<meshBasicMaterial transparent opacity={0} />
				{hovered && (
					<Html
						distanceFactor={10}
						style={{
							pointerEvents: "none",
							position: "absolute",
							top: 0,
							left: 0,
						}}
					>
						<div
							style={{
								backgroundColor: "white",
								padding: "2px",
								borderRadius: "4px",
								pointerEvents: "none",
								fontSize: "12px",
							}}
						>
							<Latex displayMode={true}>
								{`$$
									\\begin{bmatrix}
										${joints[name].matrix.elements[0].toFixed(2)} & ${joints[name].matrix.elements[4].toFixed(2)} & ${joints[name].matrix.elements[8].toFixed(2)} & ${joints[name].matrix.elements[12].toFixed(2)} \\\\
										${joints[name].matrix.elements[1].toFixed(2)} & ${joints[name].matrix.elements[5].toFixed(2)} & ${joints[name].matrix.elements[9].toFixed(2)} & ${joints[name].matrix.elements[13].toFixed(2)} \\\\
										${joints[name].matrix.elements[2].toFixed(2)} & ${joints[name].matrix.elements[6].toFixed(2)} & ${joints[name].matrix.elements[10].toFixed(2)} & ${joints[name].matrix.elements[14].toFixed(2)} \\\\
										${joints[name].matrix.elements[3].toFixed(2)} & ${joints[name].matrix.elements[7].toFixed(2)} & ${joints[name].matrix.elements[11].toFixed(2)} & ${joints[name].matrix.elements[15].toFixed(2)}
									\\end{bmatrix}
								$$`}
							</Latex>
						</div>
					</Html>
				)}
			</mesh>
		</>
	);
}

export default function App() {
	const orbitRef = useRef(null);
	return (
		<>
			<Canvas shadows camera={{ position: [0, 6, 0], fov: 60 }}>
				<ambientLight intensity={0.5} />
				<directionalLight
					position={[0, 10, 0]}
					intensity={1}
					castShadow
					shadow-mapSize-width={1024}
					shadow-mapSize-height={1024}
				/>

				{/* <pointLight position={[-10, 10, -10]} intensity={0.5} /> */}
				{/*  */}
				<URDFModel orbitRef={orbitRef} />
				{/*  */}
				<Grid
					position={[0, -0.01, 0]}
					args={[10.5, 10.5]}
					cellSize={0.5}
					cellThickness={0.5}
					cellColor="#6f6f6f"
					sectionSize={3}
					sectionThickness={1}
					sectionColor="#9d4b4b"
					infiniteGrid
					receiveShadow
				/>
				<mesh
					rotation-x={-Math.PI / 2}
					position={[0, 0, 0]}
					receiveShadow
				>
					<planeGeometry args={[10, 10]} />
					<shadowMaterial opacity={0.5} />
				</mesh>
				{/*  */}
				<OrbitControls ref={orbitRef} />
				{/*  */}
				<Environment preset="city" />
			</Canvas>
		</>
	);
}
