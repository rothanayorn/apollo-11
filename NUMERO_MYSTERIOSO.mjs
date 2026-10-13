// # ***********************************************************************
// # ERECT GUIDANCE-STABLE MEMBER TRANSFORMATION MATRIX
// # ***********************************************************************
//
// Modern JavaScript port of the AGC interpreter block at lines 660-682 of
//   Luminary099/LUNAR_LANDING_GUIDANCE_EQUATIONS.agc
// (Luminary 1A build 099 - the Lunar Module's Apollo Guidance Computer for Apollo 11).
// The original AGC comments are preserved word for word below, marked with "// #".
//
// Original block, transcribed verbatim from the .agc file:
//
//		TC	INTPRETX
//		VLOAD	UNIT
//			LAND
//		STODL	CG
//			TTF/8
//		DMP*	VXSC
//			GAINBRAK,1	# NUMERO MYSTERIOSO
//			ANGTERM
//		VAD
//			LAND
//		VSU	RTB
//			R
//			NORMUNIT
// # Page 811
//		VXV	RTB
//			LAND
//			NORMUNIT
//		STOVL	CG +6		# SECOND ROW
//			CG
//		VXV	VSL1
//			CG +6
//		STORE	CG +14
//		EXIT

// --- Tiny 3-D vector helpers (the AGC's interpreter worked on 3-component
//     vectors of double-precision words; here we use plain number triples). --

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (v, s) => [v[0] * s, v[1] * s, v[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
	a[1] * b[2] - a[2] * b[1],
	a[2] * b[0] - a[0] * b[2],
	a[0] * b[1] - a[1] * b[0],
];
const norm = (v) => Math.hypot(v[0], v[1], v[2]);
const unit = (v) => scale(v, 1 / norm(v));

/**
 * Erects the Guidance-Stable-Member transformation matrix `CG` for the
 * lunar-landing braking phase (AGC routine CGCALC).
 *
 * The 3x3 matrix `CG` rotates vectors from the stable-member frame (the IMU
 * platform axes) into the guidance frame (vertical / downrange / cross-range),
 * so that the guidance equations can work with scalars RGU, VGU, etc.
 *
 * @param {object} args
 * @param {[number, number, number]} args.LAND     - landing-site position vector (stable-member frame)
 * @param {[number, number, number]} args.R        - vehicle position vector (stable-member frame)
 * @param {[number, number, number]} args.ANGTERM  - velocity relative to lunar surface: V + R x WM
 * @param {number}                   args.TTF8     - time-to-go / 8  (TTF/8)
 * @param {number}                   args.GAINBRAK - braking-phase gain  B(2)  (GAINBRAK)
 * @returns {[number, number, number, number, number, number, number, number, number]}
 *   The 3x3 matrix, row-major: [row1 | row2 | row3] (AGC addresses CG, CG+6, CG+14).
 */
export function erectGuidanceStableMemberMatrix({ LAND, R, ANGTERM, TTF8, GAINBRAK }) {
	//		TC	INTPRETX
	//		VLOAD	UNIT
	//			LAND
	//		STODL	CG
	//			TTF/8
	//
	// VLOAD UNIT LAND : load LAND into the accumulator, normalize it  -> row 1.
	// STODL CG        : store that unit vector as the first row of CG, then
	//                   load TTF/8 (a scalar) into the accumulator.
	const row1 = unit(LAND);

	//		DMP*	VXSC
	//			GAINBRAK,1	# NUMERO MYSTERIOSO
	//			ANGTERM
	//
	// Two packed interpreter ops on one instruction word:
	//   DMP*  GAINBRAK,1 : scalar TTF/8 *= braking gain B(2)   ->  TTF/8 * GAINBRAK
	//   VXSC  ANGTERM    : the vector ANGTERM *= that scalar    ->  ANGTERM * (TTF/8 * GAINBRAK)
	const scaledTerm = scale(ANGTERM, TTF8 * GAINBRAK); // # NUMERO MYSTERIOSO

	//		VAD
	//			LAND
	//		VSU	RTB
	//			R
	//			NORMUNIT
	//
	// VAD LAND        : accumulator += LAND
	// VSU R, NORMUNIT : accumulator -= R, then unitize (RTB calls the NORMUNIT
	//                   basic subroutine and resumes the interpreter).
	//
	// The result is the tilted "line of sight" unit vector
	//     unit( LAND + GAINBRAK*(TTF/8)*ANGTERM - R )
	// whose direction aims the guidance frame so the horizontal velocity is
	// nulled out over the remaining time-to-go.  This is where the
	// "mysterious number" B(2) does its work.
	const temp = unit(sub(add(LAND, scaledTerm), R));

	// # Page 811
	//		VXV	RTB
	//			LAND
	//			NORMUNIT
	//
	// VXV LAND, NORMUNIT : accumulator = unit( temp x LAND ).  This vector is
	//                      perpendicular to LAND, so it is the horizontal
	//                      reference axis of the guidance frame.
	const row2 = unit(cross(temp, LAND)); // # SECOND ROW

	//		STOVL	CG +6		# SECOND ROW
	//			CG
	//		VXV	VSL1
	//			CG +6
	//		STORE	CG +14
	//		EXIT
	//
	// STOVL CG +6 : store row 2 into the matrix, then load row 1.
	// VXV   CG +6 : accumulator = row1 x row2  (VSL1 rescales by 2 in the AGC
	//               to keep binary precision; unnecessary with JS doubles).
	// STORE CG+14 : store as the third row, completing the orthonormal triad.
	const row3 = cross(row1, row2);

	return [...row1, ...row2, ...row3];
}


// --- Demo / sanity check (only runs when this file is executed directly) ---
// Verify that the erected matrix is orthonormal (each row a unit vector, rows
// mutually perpendicular) - i.e. that it really is a rotation matrix.

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
	const CG = erectGuidanceStableMemberMatrix({
		LAND: [1.0, 0.1, 0.0],
		R: [0.9, 0.15, 0.05],
		ANGTERM: [-0.01, 0.05, 0.0], // velocity relative to the surface
		TTF8: 10, // TTF/8 (scaled time-to-go)
		GAINBRAK: 0.4, // # NUMERO MYSTERIOSO  (braking-phase gain B(2))
	});

	const rows = [
		[CG[0], CG[1], CG[2]],
		[CG[3], CG[4], CG[5]],
		[CG[6], CG[7], CG[8]],
	];

	console.log("CG (guidance-stable-member transformation matrix):");
	rows.forEach((r) => console.log(`  [ ${r.map((x) => x.toFixed(6).padStart(10)).join(", ")} ]`));

	const unitLengths = rows.map((r) => Math.hypot(...r));
	const orthogonality = [
		dot(rows[0], rows[1]),
		dot(rows[1], rows[2]),
		dot(rows[2], rows[0]),
	];

	console.log("\nRow lengths (should be 1):", unitLengths.map((n) => n.toFixed(8)));
	console.log("Row dot products (should be ~0):", orthogonality.map((d) => d.toExponential(3)));

	const ok =
		unitLengths.every((n) => Math.abs(n - 1) < 1e-9) &&
		orthogonality.every((d) => Math.abs(d) < 1e-9);
	console.log(ok ? "\nOK: CG is orthonormal (a proper rotation matrix)." : "\nFAIL: CG is not orthonormal.");
}

