#region Header
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace SpaceEngineersScripting
{
	public class Wrapper
	{
		static void Main()
		{
			new Program().Main ("");
		}
	}


	class Program : Sandbox.ModAPI.Ingame.MyGridProgram
	{
		#endregion
		#region CodeEditor
		//Configuration
		//--------------------

		static readonly MyBlockOrientation
			shipOrientation = new MyBlockOrientation
				(Base6Directions.Direction.Forward, Base6Directions.Direction.Up);

		const uint
			phaseCount = 3;

		const float
		    baseRpm = 2.0f,
			maxRpm = 3.0f;

		const float
			drivekP = (phaseCount / Pi) * baseRpm, //constant for Drive PD controllers
			drivekD = 0.0f; //constant for Drive PD controllers

		//Expected block names are of the form
		//<prefix> <Descriptor>

		const string
			namePrefixDrive = "Rotor",      //prefix shared by all drive rotors
			nameDescriptorDriveBase = "Drive";  //name shared by all drive rotors
		static readonly string[]
			nameDesciptorDriveSides = { "L", "R" }, // {Left, Right} sides
			nameDesciptorDriveIds = { "F", "B" };   //Ids for each rotor in the column
			//Expect Drive Descriptor of the form
			//  <DescriptorDrive> ::= <base>-<side>-<Column>-<id>
			//  <Column> ::= <integer range 1..phaseCount>
			//e.g. Front Right rotor in the innermost column
			//Rotor Drive-R-1-F

		const string
			nameBusCommand = "Bus Drive.Command";

		const uint
			commandsMax = 10;    //maximum number of commands to read each run


		//Definitions
		//--------------------

		//Opcodes for use as arguments
		//-commands may be issued directly
		const string
			command_Initialise = "Init";   //args: <none>

		//Bus ids
		static readonly string
			busIdLeftRpm = CommandBus.ExtendId("LeftRpm"),  // Left RPM := float
			busIdRightRpm = CommandBus.ExtendId("RightRpm");// Right RPM := float


		//Utility definitions

		const float
			Pi = MathHelper.Pi,
			PiByTwo = Pi * 0.5f,
			ThreePiByTwo = Pi * 1.5f,
			TwoPi = Pi * 2.0f;

		const float
			phaseSize = TwoPi / phaseCount;

		/// <summary>
		/// Fast normalisation to range of
		/// -pi <= angle < pi
		/// (assuming no more than one cycle out)
		/// </summary>
		static float NormaliseRadians_Pi (float angle){
			if (angle < -Pi)
				return angle +TwoPi;
			if (angle >= Pi)
				return angle -TwoPi;
			return angle;
		}

		/// <summary>
		/// Fast normalisation to range of
		/// 0 <= angle < 2*pi
		/// (assuming no more than one cycle out)
		/// </summary>
		static float NormaliseRadians_2Pi (float angle){
			if (angle < 0.0f)
				return angle +TwoPi;
			if (angle >= TwoPi)
				return angle -TwoPi;
			return angle;
		}

		static float AngleAverage_2Pi(float a, float b){
			float
				angleDifference = NormaliseRadians_Pi( b - a );
			return
				NormaliseRadians_2Pi( a +(angleDifference/2.0f) );
		}


		static readonly Base6Directions.Direction //Unit direction vectors for ship
			///shipForward = shipOrientation.TransformDirection(Base6Directions.Direction.Forward),
			///shipBackward = shipOrientation.TransformDirection(Base6Directions.Direction.Backward),
			///shipUp = shipOrientation.TransformDirection(Base6Directions.Direction.Up),
			shipDown = shipOrientation.TransformDirection(Base6Directions.Direction.Down),
			///shipLeft = shipOrientation.TransformDirection(Base6Directions.Direction.Left),
			shipRight = shipOrientation.TransformDirection(Base6Directions.Direction.Right);

		private const uint
			driveLeft = 0,
			driveRight = 1;

		static readonly uint
			driveIdCount = (uint)nameDesciptorDriveIds.Length,
			driveCount = 2 * phaseCount * driveIdCount;  //Left-Right, phase, id

		static string NameRotorDrive(uint indexSide, uint indexColumn, uint indexId) {
			return
				namePrefixDrive +' ' +nameDescriptorDriveBase +'-'
				+nameDesciptorDriveSides[indexSide] +'-'
				+(indexColumn +1).ToString() +'-'
				+nameDesciptorDriveIds[indexId];
		}

		/// <summary>
		/// Converts drive identifiers into a single index.
		/// </summary>
		/// <remarks>
		/// Workaround for SE not providing permissions to use multi-dimensional arrays
		/// of system types.
		/// </remarks>
		/// <returns>The index to use for a Drive component.</returns>
		/// <param name="indexSide">The index of the side (<see cref="driveLeft"/>, <see cref="driveRight"/></param>
		/// <param name="indexColumn">The index of the column (<see cref="phaseCount"/>)</param>
		/// <param name="indexId">The index of the id (<see cref="nameDescriptorDriveIds"/>)</param>
		static uint IndexDrive(uint indexSide, uint indexColumn, uint indexId) {
			return indexId + driveIdCount*( indexColumn + phaseCount*( indexSide ) );
		}

		//API definitions
		public struct MotorStator
		{	//IMyMotorStator API wrapper
			//Velocity is measured internally in RPM
			//Angle is measured internally in radians. (0-2pi for unlimited rotors)
			//PROPERTIES:
			///public static void SetTorque(IMyMotorStator motorStator, float torque){
			///	motorStator.SetValue<float>("Torque", torque);
			///}
			///public static void SetBrakingTorque(IMyMotorStator motorStator, float brakingTorque){
			///	motorStator.SetValue<float>("BrakingTorque", brakingTorque);
			///}
			public static void SetVelocity(IMyMotorStator motorStator, float velocity){
				motorStator.SetValue<float>("Velocity", velocity);
			}
			public static void SetLowerLimit(IMyMotorStator motorStator, float lowerLimit){
				motorStator.SetValue<float>("LowerLimit", lowerLimit);
			}
			public static void SetUpperLimit(IMyMotorStator motorStator, float upperLimit){
				motorStator.SetValue<float>("UpperLimit", upperLimit);
			}
			///public static void SetDisplacement(IMyMotorStator motorStator, float displacement){
			///	motorStator.SetValue<float>("Displacement", displacement);
			///}
			///public static void SetWeldSpeed(IMyMotorStator motorStator, float weldSpeed){
			///	motorStator.SetValue<float>("Weld Speed", weldSpeed);
			///}
			///public static void SetForceWeld(IMyMotorStator motorStator, bool forceWeld){
			///	motorStator.SetValue<bool>("Force Weld", forceWeld);
			///}
		}


		//Internal Types
		//--------------------

		/// <summary>
		/// A bus specialising in transfer of temporary records to a single reader.
		/// </summary>
		public struct CommandBus
		{
			//The bus stores a series of records as a string, each ended by the terminator
			//	*It is not checked that records do not contain the terminator
			//	 so make sure that you do not corrupt it by using this character

			//The Command Bus stores only one Record Type:
			//-Temporary: appended on write (duplicates allowed), destructive read (FIFO)
			//e.g. use temporary records to issue commands / directional data transfer

			//Records have an Id allowing basic discrimination
			//(e.g. source and/or destination and/or data as needed)

			//Additionally, records all have a Data Type to allow for basic type checking.
			//(new types may be easily added, so long as they can be encoded/decoded
			// from a string, and ensured not to contain the record terminator)

			//Temporary records are appended to the end of the store.
			//Interpreting the read records is down to the application.

			//FORMAT
			//<Record> ::= <RecordBody><recordTerminator>
			//<RecordBody> ::= <RecordTemporary>
			//<RecordTemporary> ::= <id><Data>
			//<id> ::= <string-lengthId>
			//<Data> ::= <DataInt> | <DataFloat> | <DataString>
			//<DataInt> ::= <dataTypeInt><int>
			//<DataFloat> ::= <dataTypeFloat><float>
			//<DataString> ::= <dataTypeFloat><string>

			//e.g. Temporary string record 'Speed' = "reset"
			//Speed__Sreset\n


			//Configuration
			public const char
				recordTerminator = '\n';//'\x1E'; //Record separator

			public const char
				dataTypeInt = 'I',
				dataTypeFloat = 'F',
				dataTypeString = 'S';

			public const int
				lengthId = 8;

			//The source of the storage
			public IMyTextPanel
				bus;

			//Internal storage interface
			private string Store{
				get { return bus.GetPrivateText(); }
				set { bus.WritePrivateText(value, false); }
			}


			//Internal Implementation
			private string
				store;
			private int
				readPos;


			//PUBLIC INTERFACE

			public CommandBus(IMyTextPanel bus){
				this.bus = bus;

				store = null;
				readPos = 0;
			}

			/// <summary>
			/// Utility function to pad a string into an identifier of the required length.
			/// It is NOT checked that the given id not already too long, and hence invalid.
			/// </summary>
			/// <returns>The valid identifier based on <paramref name="id"/>.</returns>
			public static string ExtendId(string id){
				return id.PadRight(lengthId, ' ');
			}

			/// <summary>
			/// Begin the read process.
			/// Caches the store and prepares for optimised reads.
			/// </summary>
			public void BeginRead(){
				store = Store;
				readPos = 0;
			}

			/// <summary>
			/// Reads the next record in the cached store.
			/// </summary>
			/// <returns><c>true</c>, if a record was found; <c>false</c> if were no records to read.</returns>
			public bool ReadNext(out string id, out char dataType, out string data){
				if (readPos < store.Length)
				{
					id = store.Substring (readPos, lengthId);
					readPos += lengthId;

					dataType = store[readPos++];

					int dataPos = readPos;
					while (store[readPos++] != recordTerminator) {};
					data = store.Substring (dataPos, readPos -dataPos -1);

					return true;
				} else {
					//end of storage; no record to return
					id = null;
					dataType = '\0';
					data = null;
					return false;
				}
			}

			/// <summary>
			/// End the read process.
			/// Saves the cached store.
			/// </summary>
			public void EndRead(){
				//no work required if nothing was read
				if (readPos > 0) {
					Store = store.Remove(0, readPos);
					readPos = 0;
				}
				store = null;
			}

		}

		/// <summary>
		/// A cut-down PID controller.
		/// Handles Proportional and Differential feedback only.
		/// Only supports a target of 0.
		/// </summary>
		public struct PD_Zero
		{
			private float
				kP, kD;

			private float
				errorLast;

			public void Reset()
			{
				errorLast = 0;
			}

			public float Update(float measurement, float elapsedSeconds_reciprocal)
			{
				//calculate current error (inverted to provide negative feedback)
				float error = -measurement;

				float result =
					(kP * error)
					+(kD * ( (error -errorLast) * elapsedSeconds_reciprocal ));

				errorLast = error;

				return result;
			}

			public PD_Zero(float kP, float kD)
			{
				this.kP = kP;
				this.kD = kD;

				this.errorLast = 0;
			}

			public string Store()
			{
				return
					errorLast.ToString("R");
			}

			public bool TryRestore(string storage)
			{
				return
					float.TryParse (storage, out errorLast);
			}
		}

		public struct Status
		{
			//program data not persistent across restarts
			public bool
				initialised;

			//status data persistent across restarts
			public float
				leftRpm, rightRpm;

			public PD_Zero[]
				driveControllers;


			//configuration constants
			private const char delimiter = ';';

			//Operations

			public void Initialise()
			{   //data setup
				driveControllers = new PD_Zero[driveCount];
				for (uint i=0; i<driveCount; i++) {
					driveControllers[i] = new PD_Zero (drivekP, drivekD);
				}
				leftRpm = 0.0f;
				rightRpm = 0.0f;
			}

			public string Store()
			{
				StringBuilder s = new StringBuilder();
				for (uint i=0; i<phaseCount; i++) {
					for (uint ii=0; ii<nameDesciptorDriveIds.Length; ii++) {
						s.Append( driveControllers[IndexDrive(driveLeft,i,ii)].Store() );
						s.Append( delimiter );
						s.Append( driveControllers[IndexDrive(driveRight,i,ii)].Store() );
						s.Append( delimiter );
					}
				}
				s.Append( leftRpm.ToString("R") );
				s.Append( delimiter );
				s.Append( rightRpm.ToString("R") );

				return s.ToString();
			}

			public bool TryRestore(string storage)
			{
				string[] elements = storage.Split(delimiter);
				if ( !(elements.Length == driveCount +2) )
					return false;

				uint pos = 0;    //element to examine

				driveControllers = new PD_Zero[driveCount];
				for (uint i=0; i<phaseCount; i++) {
					for (uint ii=0; ii<driveIdCount; ii++) {
						if ( !(
							driveControllers[IndexDrive(driveLeft,i,ii)].TryRestore (elements[pos++])
							&& driveControllers[IndexDrive(driveRight,i,ii)].TryRestore (elements[pos++])
						))
							return false;
					}
				}

				return
					float.TryParse (elements[pos++], out leftRpm)
					&& float.TryParse (elements[pos++], out rightRpm);
			}
		}

		/// <summary>
		/// Holds the data and operations dynamically associated with each Drive
		/// </summary>
		public struct DriveConfig
		{
			public IMyMotorStator
				drive;
			public bool
				inverted;
			public float
				offset; //observed angle + offset = true angle (from zero; may need inverting)

			/// <summary>
			/// The actual angle of the rotor, accounting for inversion and offset.
			/// 0 => shipDown
			/// Positive direction is clockwise when looking shipLeft (moves vehicle forward)
			/// </summary>
			/// <value>The true angle in radians, range -2pi..2pi (not normalised)</value>
			public float
				TrueAngle {
					get {
						if (inverted) { return -(drive.Angle +offset); }
						else          { return   drive.Angle +offset; }
					}
				}

			//Operations

			public bool Evaluate()
			{
				//check whether the 'Up' side of the rotor is facing shipRight or shipLeft
				switch (drive.Orientation.TransformDirectionInverse(shipRight)) {
					case Base6Directions.Direction.Up:
						inverted = false;
						break;
					case Base6Directions.Direction.Down:
						inverted = true;
						break;
					default:
						inverted = false;
						offset = float.NaN;
						return false;
				}
				//'0' position is the rotor's 'Right', which should be facing shipDown.
				switch (drive.Orientation.TransformDirectionInverse(shipDown)) {
					case Base6Directions.Direction.Right:
						offset = 0.0f;
						break;
					case Base6Directions.Direction.Forward:
						offset = -ThreePiByTwo;
						break;
					case Base6Directions.Direction.Left:
						offset = -Pi;
						break;
					case Base6Directions.Direction.Backward:
						offset = -PiByTwo;
						break;
					default:
						offset = float.NaN;
						return false;
				}
				return true;
			}
		}

		//Global variables
		//--------------------
		CommandBus
			busCmd;

		Status
			status;

		DriveConfig[]
			drives = new DriveConfig[driveCount];

		float[] //phases, 0..2pi
			phaseDrives = new float[driveCount],
			phaseColumns = new float[phaseCount*2], //Left-phases, Right-phases
			phaseSides = new float[2];


		//Program
		//--------------------

		public Program()
		{
			Echo ("Restarted.");

			//script has been reloaded
			//-may be first time running
			//-world may have been reloaded (or script recompiled)
			if (Storage == null) {
				//use default values
				status.Initialise();
			} else {
				//attempt to restore saved values
				//  -otherwise use defaults
				Echo ("restoring saved state...");
				if ( !status.TryRestore(Storage) ){
					Echo ("restoration failed.");
					status.Initialise();
				}
			}
			//We are not initialised after restart
			//-attempt to initialise now to reduce load at run-time
			status.initialised = false;
			Initialise();

//			Echo ("fwd: " +shipForward.ToString());
//			Echo ("up: " +shipUp.ToString());
//			Echo ("right: " + shipRight.ToString ());
		}


		public void Save()
		{
			//Command data should be persistent across recompilation
			//-must save after each execution, so no need to save here
			//Storage = status.Store();
		}


		public void Main(string argument)
		{
			//First ensure the system is able to process commands
			//-if necessary, perform first time setup
			//-if necessary or requested, initialise the system
			if ( !status.initialised || argument == command_Initialise) {
				//if we cannot initialise, end here
				if ( !Initialise() )
					return;
			}
			if (argument == command_Initialise) {
				Echo ("resetting.");
				status.Initialise ();
			}
			else if ( !Validate() ) {
				//if saved state is not valid, try re-initialising
				//if we cannot initialise, end here
				if ( !Initialise() )
					return;
			}

			//Read any control commands
			busCmd.BeginRead();
			ReadCommands();
			busCmd.EndRead();

			//Perform main processing
			Update ();

			//Save status back
			Storage = status.Store();
		}


		/// <summary>
		/// Parses an RPM value, and sets the target if it is valid.
		/// </summary>
		/// <returns><c>true</c> if rpm was set, <c>false</c> otherwise.</returns>
		private bool ParseRpm(string s, ref float target ){
			float rpm;
			if (float.TryParse (s, out rpm)) {
				if ( (rpm >= -baseRpm) && (rpm <= baseRpm) ) {
					target = rpm;
					return true;
				} else {
					Echo ("WARNING: RPM value out of bounds \"" +s +"\"");
					return false;
				}
			} else {
				Echo ("WARNING: Invalid RPM value \"" +s +"\"");
				return false;
			}
		}

		private void ReadCommands()
		{
			string
				id,	data;
			char
				dataType;

			//Read commands until there are none left, or hit max
			for (int i=0; i<commandsMax; i++) {
				if (busCmd.ReadNext (out id, out dataType, out data)) {
					switch (dataType) {
						case CommandBus.dataTypeString:

							break;

						case CommandBus.dataTypeFloat:
							if (id == busIdLeftRpm) {
								if (ParseRpm (data, ref status.leftRpm)) {
									Echo ("Left RPM set.");
								}
							} else if (id == busIdRightRpm) {
								if (ParseRpm (data, ref status.rightRpm)) {
									Echo ("Right RPM set.");
								}
							} else {
								break;
							}
							continue;

						default:
							break;
					}
					//If we exited without 'continue', the command was unrecognised.
					Echo ("WARNING: Unrecognised command \"" +id +"\" type '" +dataType +"'");
				} else {
					Echo ("All commands read.");
					return;
				}
			}

			Echo ("Not reading further commands.");
		}


		private void ObservePhases()
		{
			//The actual phase is defined by the weight-bearing leg
			//This is whichever drive one has the angle closest to zero
			//-(potentially untrue if there is roll, pitch, or uneven ground, but good enough)

			for (uint i=0; i<2; i++) {  //Left-Right
				float
					phaseOffset = 0.0f;
				float
					sidePhase = float.NaN,
					sideAbsMin = float.PositiveInfinity;

				for (uint ii=0; ii<phaseCount; ii++) {  //Column
					float
						columnPhase = float.NaN,
						columnAbsMin = float.PositiveInfinity;

					for (uint iii=0; iii<driveIdCount; iii++) { //Id
						uint
							indexDrive = IndexDrive(i,ii,iii);
						float
							angle = drives[indexDrive].TrueAngle,
							phase = NormaliseRadians_2Pi(angle +phaseOffset),
							abs = Math.Abs( NormaliseRadians_Pi(angle) );

						phaseDrives[indexDrive] = phase;

						if (abs < columnAbsMin) {
							columnAbsMin = abs;
							columnPhase = phase;
						}
					}

					phaseColumns[ (i*phaseCount) +ii ] = columnPhase;

					if (columnAbsMin < sideAbsMin) {
						sideAbsMin = columnAbsMin;
						sidePhase = columnPhase;
					}

					phaseOffset += phaseSize;
				}

				phaseSides[i] = sidePhase;
			}
		}

		private void SetupDrives(uint side, float phaseTarget, float elapsedSeconds_reciprocal, float rpmSide){
			for (uint ii=0; ii<phaseCount; ii++) {  //column
				for (uint iii=0; iii<driveIdCount; iii++) { //id
					uint indexDrive = IndexDrive(side, ii, iii);
					DriveConfig drive = drives[indexDrive];

					//calculate target angle based on phase
					float //0..2pi
						target = NormaliseRadians_2Pi (phaseTarget -(ii * phaseSize));
					float //-pi..pi
						error = NormaliseRadians_Pi (NormaliseRadians_Pi(drives[indexDrive].TrueAngle) -target);

					float
						correction = status.driveControllers[indexDrive].Update(error, elapsedSeconds_reciprocal);
					float
						velocity = MyMath.Clamp (rpmSide + correction, 0.0f, maxRpm);

					MotorStator.SetVelocity (drive.drive,
						drive.inverted ? -velocity : velocity);
				}
			}
		}

		private void Update()
		{
			double elapsedSeconds = Runtime.TimeSinceLastRun.TotalSeconds;
			//precalculate reciprocal for optimisation
			//-handle special case of time=0
			float elapsedSeconds_reciprocal = (elapsedSeconds == 0.0) ? 0.0f : 1.0f / (float)elapsedSeconds;

			Echo ("dt = " +elapsedSeconds.ToString() );

//			for (uint i=0; i<2; i++) {
//				for (uint ii=0; ii<phaseCount; ii++) {
//					for (uint iii=0; iii<driveIdCount; iii++) {
//						DriveConfig config = drives[IndexDrive(i, ii, iii)];
//
//						Echo (config.drive.CustomName +" :");
//						Echo (" inverted = " +config.inverted.ToString() );
//						Echo (" offset = " +MathHelper.ToDegrees(config.offset).ToString("F1") +'\u00B0');
//						//Echo (" angle = " + MathHelper.ToDegrees(config.drive.TrueAngle).ToString("F1") +'\u00B0');
//					}
//				}
//			}
			ObservePhases();

			//Update PDs and adjust Drives
			float   //phases, 0..2pi
				phaseTargetLeft,
				phaseTargetRight;

			//if possible, adjust targets to synchronise left and right sides
			if (status.leftRpm == status.rightRpm) {
				Echo ("synchronising sides...");
				float phaseTargetShared =
						//AngleAverage_2Pi(phaseSides[driveLeft], phaseSides[driveRight]);
						AngleAverage_2Pi(phaseColumns[0], phaseColumns[phaseCount]);
//				Echo ("TARGET: " +MathHelper.ToDegrees(phaseTargetShared).ToString("F1") +'\u00B0');
				phaseTargetLeft = phaseTargetShared;
				phaseTargetRight = phaseTargetShared;
			} else {
				phaseTargetLeft = phaseSides[driveLeft];
				phaseTargetRight = phaseSides[driveRight];
			}

			//{!}TODO adjust to synchronise within sides, within columns

			//set up Drives
			SetupDrives(driveLeft, phaseTargetLeft, elapsedSeconds_reciprocal, status.leftRpm);
			SetupDrives(driveRight, phaseTargetRight, elapsedSeconds_reciprocal, status.rightRpm);

			//Echo status
//			Echo ("phaseLeft: " +MathHelper.ToDegrees(phaseSides[0]).ToString("F1") +'\u00B0');
//			for (uint i=0; i<phaseCount; i++) {
//				Echo ("  column" +(i+1).ToString() +": " +MathHelper.ToDegrees(phaseColumns[i]).ToString("F1") +'\u00B0');
//				for (uint ii=0; ii<driveIdCount; ii++){
//					Echo ("    id" +(nameDesciptorDriveIds[ii]) +": " +MathHelper.ToDegrees(phaseDrives[IndexDrive(driveLeft,i,ii)]).ToString("F1") +'\u00B0');
//					Echo ("     rpm: " +drives[IndexDrive(driveLeft,i,ii)].drive.Velocity.ToString("F1") +"rpm");
//				}
//			}
//			Echo ("phaseRight: " +MathHelper.ToDegrees(phaseSides[1]).ToString("F1") +'\u00B0');
//			for (uint i=0; i<phaseCount; i++) {
//				Echo ("  column" +(i+1).ToString() +": " +MathHelper.ToDegrees(phaseColumns[phaseCount +i]).ToString("F1") +'\u00B0');
//				for (uint ii=0; ii<driveIdCount; ii++){
//					Echo ("    id" +(nameDesciptorDriveIds[ii]) +": " +MathHelper.ToDegrees(phaseDrives[IndexDrive(driveRight,i,ii)]).ToString("F1") +'\u00B0');
//					Echo ("     rpm: " +drives[IndexDrive(driveRight,i,ii)].drive.Velocity.ToString("F1") +"rpm");
//				}
//			}
//
		}


		private bool FindBlock<BlockType>(out BlockType block, string nameBlock, ref List<IMyTerminalBlock> temp)
			where BlockType : class, IMyTerminalBlock
		{
			block = null;
			GridTerminalSystem.GetBlocksOfType<BlockType> (temp);
			for (int i=0; i<temp.Count; i++){
				if (temp[i].CustomName == nameBlock) {
					if (block == null) {
						block = (BlockType)temp[i];
					} else {
						Echo ("ERROR: duplicate name \"" +nameBlock +"\"");
						return false;
					}
				}
			}
			//verify that the block was found
			if (block == null) {
				Echo ("ERROR: block not found \"" +nameBlock +"\"");
				return false;
			}
			return true;
		}

		private bool ValidateBlock(IMyTerminalBlock block, bool callbackRequired=false)
		{
			//check for block deletion?

			//check that we have required permissions to control the block
			if ( ! Me.HasPlayerAccess(block.OwnerId) ) {
				Echo ("ERROR: no permissions for \"" +block.CustomName +"\"");
				return false;
			}

			//check that the block has required permissions to make callbacks
			if ( callbackRequired && !block.HasPlayerAccess(Me.OwnerId) ) {
				Echo ("ERROR: no permissions on \"" +block.CustomName +"\"");
				return false;
			}

			//check that block is functional
			if (!block.IsFunctional) {
				Echo ("ERROR: non-functional block \"" +block.CustomName +"\"");
				return false;
			}

			return true;
		}

		private bool ValidateDrive(IMyMotorStator drive)
		{
			if ( ! ValidateBlock(drive) )
				return false;

			if ( ! drive.IsAttached ){
				Echo ("ERROR: Drive not attached \"" +drive.CustomName +"\"");
				return false;
			}

			return true;
		}

		private bool Initialise()
		{
			status.initialised = false;
			Echo ("initialising...");

			var temp = new List<IMyTerminalBlock>();

			//Discover drive rotors
			GridTerminalSystem.GetBlocksOfType<IMyMotorStator>(temp);
			for (uint i=0; i<2; i++) {  //Left-Right
				for (uint ii=0; ii<phaseCount; ii++) {  //Column
					for (uint iii=0; iii<driveIdCount; iii++) { //Id
						//Clear any existing reference
						uint indexDrive = IndexDrive(i, ii, iii);
						drives[indexDrive].drive = null;

						//Find the required Drive
						string nameBlock = NameRotorDrive(i,ii,iii);
						for (int t=0; t<temp.Count; t++) {
							if (temp[t].CustomName == nameBlock) {
								if (drives[indexDrive].drive == null) {
									drives[indexDrive].drive = (IMyMotorStator)temp[t];
								} else {
									Echo ("ERROR: duplicate name \"" + nameBlock + "\"");
									return false;
								}
							}
						}
						//Check that the Drive was found
						if (drives[indexDrive].drive == null) {
							Echo ("ERROR: block not found \"" +nameBlock +"\"");
							return false;
						}
						//Check that the found Drive is operable
						if ( !ValidateDrive(drives[indexDrive].drive) )
							return false;
						if ( !drives[indexDrive].Evaluate() ) {
							Echo ("ERROR: drive has invalid orientation \"" + nameBlock + "\"");
							return false;
						}

					}
				}
			}

			//Discover command bus
			{
				IMyTextPanel textPanel;
				if ( !( FindBlock<IMyTextPanel>(out textPanel, nameBusCommand, ref temp)
				        && ValidateBlock(textPanel, callbackRequired:false) ))
					return false;
				else
					busCmd = new CommandBus(textPanel);
			}

			status.initialised = true;
			Echo ("Initialisation completed with no errors.");
			return true;
		}


		private bool Validate()
		{
			bool valid =
				ValidateBlock(busCmd.bus);

			for (uint i=0; i<driveCount; i++) {
				valid = valid
					& ValidateDrive(drives[i].drive);
			}

			if ( !valid ) {
				Echo ("Validation of saved blocks failed.");
			}
			return valid;
		}
		#endregion
		#region footer
	}
}
#endregion