// Copyright (c) 2013-2016 Robert Rouhani <robert.rouhani@gmail.com> and other contributors (see CONTRIBUTORS file).
// Licensed under the MIT License - https://raw.github.com/Robmaister/SharpNav/master/LICENSE

using System;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Collections.Generic;
using System.Numerics;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;

using SharpNav;
using SharpNav.Crowds;
using SharpNav.Geometry;
using SharpNav.IO.Json;
using SharpNav.Pathfinding;

using Key = OpenTK.Input.Key;
using Vector4 = OpenTK.Vector4;

namespace SharpNav.Examples
{
	public partial class ExampleWindow : GameWindow
	{
		private enum DisplayMode : int
		{
			None,
			Heightfield,
			CompactHeightfield,
			DistanceField,
			Regions,
			Contours,
			PolyMesh,
			PolyMeshDetail,
			NavMesh,
			Pathfinding,
			MAX
		}

		private bool interceptExceptions;

		private Camera cam;
		private float zoom = MathF.PI / 4;

		//Generate poly mesh
		private Heightfield heightfield;
		private CompactHeightfield compactHeightfield;
		private Color4[] regionColors;
		private ContourSet contourSet;
		private PolyMesh polyMesh;
		private PolyMeshDetail polyMeshDetail;
		
		//Pathfinding
		//private NavMeshCreateParams parameters;
		private NavMeshBuilder buildData;
		private TiledNavMesh tiledNavMesh;
		private NavMeshQuery navMeshQuery;

		//Smooth path for a single unit
		private NavPoint startPt;
		private NavPoint endPt;
		private Path path;
		private List<System.Numerics.Vector3> smoothPath;

		//A crowd is made up of multiple units, each with their own path
		private Crowd crowd;
		private const int MAX_AGENTS = 128;
		private const int AGENT_MAX_TRAIL = 64;
		private int numIterations = 50;
		private int numActiveAgents = 0;
		private AgentTrail[] trails = new AgentTrail[MAX_AGENTS];

		private struct AgentTrail
		{
			public System.Numerics.Vector3[] Trail;
			public int HTrail;
		}

		private bool hasGenerated;
		private bool displayLevel = true;
		private DisplayMode displayMode = DisplayMode.Pathfinding;

		private KeyboardState prevK;
		private MouseState prevM;

		private Matrix4 gwenProjection;

		public ExampleWindow()
			: base(800, 600, new GraphicsMode(32, 8, 0, 4))
		{
			cam = new Camera();


			this.Title = "SharpNav Example";
		}

		protected override void OnLoad(EventArgs e)
		{
			base.OnLoad(e);

			InitializeOpenGL();

			LoadLevel();
			LoadDebugMeshes();


			gwenProjection = Matrix4.CreateOrthographicOffCenter(0, Width, Height, 0, -1, 1);
			
			
			InitializeUI();
		}

		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame(e);

			if (!Focused)
				return;

			KeyboardState k = OpenTK.Input.Keyboard.GetState();
			MouseState m = OpenTK.Input.Mouse.GetState();

			bool isShiftDown = false;
			if (k[Key.LShift] || k[Key.RShift])
				isShiftDown = true;

			//TODO make cam speed/shift speedup controllable from GUI
			float camSpeed = 10f * (float)e.Time * (isShiftDown ? 3f : 1f);
			float zoomSpeed = (float)Math.PI * (float)e.Time * (isShiftDown ? 0.2f : 0.1f);

			if (k[Key.W])
				cam.Move(-camSpeed);
			if (k[Key.A])
				cam.Strafe(-camSpeed);
			if (k[Key.S])
				cam.Move(camSpeed);
			if (k[Key.D])
				cam.Strafe(camSpeed);
			if (k[Key.Q])
				cam.Elevate(camSpeed);
			if (k[Key.E])
				cam.Elevate(-camSpeed);
			
			if (k[Key.PageUp])
			{
				zoom += zoomSpeed;
				if (zoom > MathF.PI / 2)
					zoom = MathF.PI / 2;
				
				Console.WriteLine($"ZOOM: {zoom}");
			}
			if (k[Key.PageDown])
			{
				zoom -= zoomSpeed;
				if (zoom < 0.002f)
					zoom = 0.002f;

				Console.WriteLine($"ZOOM: {zoom}");
			}

			
			if (m[MouseButton.Right])
			{
				cam.RotatePitch((m.X - prevM.X) * (float)e.Time * 2f);
				cam.RotateHeading((prevM.Y - m.Y) * (float)e.Time * 2f);
			}

			float aspect = Width / (float)Height;
			Matrix4 persp = Matrix4.CreatePerspectiveFieldOfView(zoom, aspect, 0.1f, 1000f);
			GL.MatrixMode(MatrixMode.Projection);
			GL.LoadMatrix(ref persp);
			GL.MatrixMode(MatrixMode.Modelview);
			cam.LoadView();
			if (crowd != null)
				crowd.Update((float)e.Time);

			prevK = k;
			prevM = m;
		}


		protected override void OnKeyPress(KeyPressEventArgs e)
		{
			if (!Focused)
				return;
			
			
		}

		protected override void OnKeyDown(KeyboardKeyEventArgs e)
		{
			if (e.Key == Key.Escape)
				Exit();
			
			
			else if (e.Key == Key.F1)
				GenerateNavMesh();
			else if (e.Key == Key.F2)
				GeneratePathfinding();
			else if (e.Key == Key.F3)
				GenerateCrowd();
			else if (e.Key == Key.F4)
			{
				var dm = (int) displayMode;
				dm = (dm + 1) % (int) (DisplayMode.MAX);
				displayMode = (DisplayMode)dm;
				Console.WriteLine($"Display Mode: {displayMode}");
			}
			else if (e.Key == Key.F5)
			{
				//Gwen.Platform.Neutral.FileSave("Save NavMesh to file", ".", "All SharpNav Files(.snb, .snx, .snj)|*.snb;*.snx;*.snj|SharpNav Binary(.snb)|*.snb|SharpNav XML(.snx)|*.snx|SharpNav JSON(.snj)|*.snj", SaveNavMeshToFile);
			}
			else if (e.Key == Key.F9)
			{
				//Gwen.Platform.Neutral.FileOpen("Load NavMesh from file", ".", "All SharpNav Files(.snb, .snx, .snj)|*.snb;*.snx;*.snj|SharpNav Binary(.snb)|*.snb|SharpNav XML(.snx)|*.snx|SharpNav JSON(.snj)|*.snj", LoadNavMeshFromFile);
			}
			else if (e.Key == Key.F11)
				WindowState = OpenTK.WindowState.Normal;
			else if (e.Key == Key.F12)
				WindowState = OpenTK.WindowState.Fullscreen;


			base.OnKeyDown(e);
		}
		
		protected override void OnRenderFrame(FrameEventArgs e)
		{
			base.OnRenderFrame(e);

			GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

			if (displayLevel)
			{
				GL.Enable(EnableCap.Lighting);
				GL.Enable(EnableCap.Light0);
				GL.Light(LightName.Light0, LightParameter.Position, new Vector4(0.5f, 1, 0.5f, 0));

				DrawLevel();

				GL.Disable(EnableCap.Light0);
				GL.Disable(EnableCap.Lighting);
			}

			switch (displayMode)
			{
				case DisplayMode.Heightfield: 
					DrawHeightfield();
					break;
				case DisplayMode.CompactHeightfield:
					DrawCompactHeightfield();
					break;
				case DisplayMode.DistanceField:
					DrawDistanceField();
					break;
				case DisplayMode.Regions:
					DrawRegions();
					break;
				case DisplayMode.Contours:
					DrawContours();
					break;
				case DisplayMode.PolyMesh:
					DrawPolyMesh();
					break;
				case DisplayMode.PolyMeshDetail:
					DrawPolyMeshDetail();
					break;
				case DisplayMode.NavMesh:
					DrawNavMesh();
					break;
				case DisplayMode.Pathfinding:
					DrawPathfinding();
					break;
			}

			DrawCrowd();
			DrawUI();

			SwapBuffers();
		}

		protected override void OnResize(EventArgs e)
		{
			base.OnResize(e);

			GL.Viewport(0, 0, Width, Height);
			float aspect = Width / (float)Height;

			Matrix4 persp = Matrix4.CreatePerspectiveFieldOfView(zoom, aspect, 0.1f, 1000f);
			GL.MatrixMode(MatrixMode.Projection);
			GL.LoadMatrix(ref persp);
			GL.MatrixMode(MatrixMode.Modelview);
			cam.LoadView();

			gwenProjection = Matrix4.CreateOrthographicOffCenter(0, Width, Height, 0, 0, 1);
		}

		protected override void OnUnload(EventArgs e)
		{
			UnloadLevel();
			UnloadDebugMeshes();

			base.OnUnload(e);
		}

		private void LoadNavMeshFromFile(string path)
		{
			try
			{

				tiledNavMesh = new NavMeshJsonSerializer().Deserialize(path);
				navMeshQuery = new NavMeshQuery(tiledNavMesh, 2048);
				hasGenerated = true;
				displayMode = DisplayMode.NavMesh;
			}
			catch (Exception e)
			{
				if (!interceptExceptions)
					throw;
				else
				{
					hasGenerated = false;
					tiledNavMesh = null;
					navMeshQuery = null;
					Console.WriteLine("Navmesh loading failed with exception:" + Environment.NewLine + e.ToString());
				}
			}
		}

		private void SaveNavMeshToFile(string path)
		{
			if (!hasGenerated)
			{
				Console.WriteLine("No navmesh generated or loaded, cannot save.");
				return;
			}

			try
			{
				new NavMeshJsonSerializer().Serialize(path, tiledNavMesh);
			}
			catch (Exception e)
			{
				if (!interceptExceptions)
					throw;
				else
				{
					Console.WriteLine("Navmesh saving failed with exception:" + Environment.NewLine + e.ToString());
					return;
				}
			}

			Console.WriteLine("Saved to file!");
		}

		private void GenerateNavMesh()
		{
			Console.WriteLine("Generating NavMesh");

			Stopwatch sw = new Stopwatch();
			sw.Start();
			long prevMs = 0;
			try
			{
				//level.SetBoundingBoxOffset(new SVector3(settings.CellSize * 0.5f, settings.CellHeight * 0.5f, settings.CellSize * 0.5f));
				var levelTris = level.GetTriangles();
				var triEnumerable = TriangleEnumerable.FromTriangle(levelTris, 0, levelTris.Length);
				BBox3 bounds = triEnumerable.GetBoundingBox();

				heightfield = new Heightfield(bounds, settings);

				Console.WriteLine("Heightfield");
				Console.WriteLine(" + Ctor\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				/*Area[] areas = AreaGenerator.From(triEnumerable, Area.Default)
					.MarkAboveHeight(areaSettings.MaxLevelHeight, Area.Null)
					.MarkBelowHeight(areaSettings.MinLevelHeight, Area.Null)
					.MarkBelowSlope(areaSettings.MaxTriSlope, Area.Null)
					.ToArray();
				heightfield.RasterizeTrianglesWithAreas(levelTris, areas);*/
				heightfield.RasterizeTriangles(levelTris, Area.Default);

				Console.WriteLine(" + Rasterization\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				Console.WriteLine(" + Filtering");
				prevMs = sw.ElapsedMilliseconds;

				heightfield.FilterLedgeSpans(settings.VoxelAgentHeight, settings.VoxelMaxClimb);

				Console.WriteLine("   + Ledge Spans\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				heightfield.FilterLowHangingWalkableObstacles(settings.VoxelMaxClimb);

				Console.WriteLine("   + Low Hanging Obstacles\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				heightfield.FilterWalkableLowHeightSpans(settings.VoxelAgentHeight);

				Console.WriteLine("   + Low Height Spans\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				compactHeightfield = new CompactHeightfield(heightfield, settings);

				Console.WriteLine("CompactHeightfield");
				Console.WriteLine(" + Ctor\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;
				 
				compactHeightfield.Erode(settings.VoxelAgentRadius);

				Console.WriteLine(" + Erosion\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				compactHeightfield.BuildDistanceField();

				Console.WriteLine(" + Distance Field\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				compactHeightfield.BuildRegions(0, settings.MinRegionSize, settings.MergedRegionSize);

				Console.WriteLine(" + Regions\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				Random r = new Random();
				regionColors = new Color4[compactHeightfield.MaxRegions];
				regionColors[0] = Color4.Black;
				for (int i = 1; i < regionColors.Length; i++)
					regionColors[i] = new Color4((byte)r.Next(0, 255), (byte)r.Next(0, 255), (byte)r.Next(0, 255), 255);

				Console.WriteLine(" + Colors\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				contourSet = compactHeightfield.BuildContourSet(settings);

				Console.WriteLine("ContourSet");
				Console.WriteLine(" + Ctor\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				polyMesh = new PolyMesh(contourSet, settings);

				Console.WriteLine("PolyMesh");
				Console.WriteLine(" + Ctor\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				polyMeshDetail = new PolyMeshDetail(polyMesh, compactHeightfield, settings);

				Console.WriteLine("PolyMeshDetail");
				Console.WriteLine(" + Ctor\t\t\t\t" + (sw.ElapsedMilliseconds - prevMs).ToString("D3") + " ms");
				prevMs = sw.ElapsedMilliseconds;

				hasGenerated = true;


			}
			catch (Exception e)
			{
				if (!interceptExceptions)
					throw;
				else
					Console.WriteLine("Navmesh generation failed with exception:" + Environment.NewLine + e.ToString());
			}
			finally
			{
				sw.Stop();
			}

			if (hasGenerated)
			{
				try
				{
					Console.WriteLine("Has generated, now pathfinding, and crowd");
					GeneratePathfinding();

					//Pathfinding with multiple units
					GenerateCrowd();
				}
				catch (Exception e)
				{
					Console.WriteLine("Pathfinding generation failed with exception" + Environment.NewLine + e.ToString());
					hasGenerated = false;
				}

				Console.WriteLine("Navmesh generated successfully in " + sw.ElapsedMilliseconds + "ms.");
				Console.WriteLine("Rasterized " + level.GetTriangles().Length + " triangles.");
				Console.WriteLine("Generated " + contourSet.Count + " regions.");
				Console.WriteLine("PolyMesh contains " + polyMesh.VertCount + " vertices in " + polyMesh.PolyCount + " polys.");
				Console.WriteLine("PolyMeshDetail contains " + polyMeshDetail.VertCount + " vertices and " + polyMeshDetail.TrisCount + " tris in " + polyMeshDetail.MeshCount + " meshes.");
			}
		}

		private void GeneratePathfinding()
		{
			if (!hasGenerated)
				return;

			Console.WriteLine("Gen Pathfinding");

			Random rand = new Random();
			NavQueryFilter filter = new NavQueryFilter();

			buildData = new NavMeshBuilder(polyMesh, polyMeshDetail, new SharpNav.Pathfinding.OffMeshConnection[0], settings);

			tiledNavMesh = new TiledNavMesh(buildData);
			navMeshQuery = new NavMeshQuery(tiledNavMesh, 2048);

			//Find random start and end points on the poly mesh
			/*int startRef;
			navMeshQuery.FindRandomPoint(out startRef, out startPos);*/

			var c = new System.Numerics.Vector3(10, 0, 0);
			var e = new System.Numerics.Vector3(5, 5, 5);
			navMeshQuery.FindNearestPoly(ref c, ref e, out startPt);

			navMeshQuery.FindRandomPointAroundCircle(ref startPt, 1000, out endPt);

			//calculate the overall path, which contains an array of polygon references
			int MAX_POLYS = 256;
			path = new Path();
			navMeshQuery.FindPath(ref startPt, ref endPt, filter, path);

			//find a smooth path over the mesh surface
			int npolys = path.Count;
			var iterPos = new System.Numerics.Vector3();
			var targetPos = new System.Numerics.Vector3();
			navMeshQuery.ClosestPointOnPoly(startPt.Polygon, startPt.Position, ref iterPos);
			navMeshQuery.ClosestPointOnPoly(path[npolys - 1], endPt.Position, ref targetPos);

			smoothPath = new List<System.Numerics.Vector3>(2048);
			smoothPath.Add(iterPos);

			float STEP_SIZE = 0.5f;
			float SLOP = 0.01f;
			while (npolys > 0 && smoothPath.Count < smoothPath.Capacity)
			{
				//find location to steer towards
				var steerPos = new System.Numerics.Vector3();
				StraightPathFlags steerPosFlag = 0;
				NavPolyId steerPosRef = NavPolyId.Null;

				if (!GetSteerTarget(navMeshQuery, iterPos, targetPos, SLOP, path, ref steerPos, ref steerPosFlag, ref steerPosRef))
					break;

				bool endOfPath = (steerPosFlag & StraightPathFlags.End) != 0 ? true : false;
				bool offMeshConnection = (steerPosFlag & StraightPathFlags.OffMeshConnection) != 0 ? true : false;

				//find movement delta
				var delta = steerPos - iterPos;
				float len = (float)Math.Sqrt(System.Numerics.Vector3.Dot(delta, delta));

				//if steer target is at end of path or off-mesh link
				//don't move past location
				if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
					len = 1;
				else
					len = STEP_SIZE / len;

				var moveTgt = new System.Numerics.Vector3();
				VMad(ref moveTgt, iterPos, delta, len);

				//move
				var result = new System.Numerics.Vector3();
				List<NavPolyId> visited = new List<NavPolyId>(16);
				NavPoint startPoint = new NavPoint(path[0], iterPos);
				navMeshQuery.MoveAlongSurface(ref startPoint, ref moveTgt, out result, visited);
				path.FixupCorridor(visited);
				npolys = path.Count;
				float h = 0;
				navMeshQuery.GetPolyHeight(path[0], result, ref h);
				result.Y = h;
				iterPos = result;

				//handle end of path when close enough
				if (endOfPath && InRange(iterPos, steerPos, SLOP, 1.0f))
				{
					//reached end of path
					iterPos = targetPos;
					if (smoothPath.Count < smoothPath.Capacity)
					{
						smoothPath.Add(iterPos);
					}
					break;
				}

				//store results
				if (smoothPath.Count < smoothPath.Capacity)
				{
					smoothPath.Add(iterPos);
				}
			}
		}

		/// <summary>
		/// Scaled vector addition
		/// </summary>
		/// <param name="dest">Result</param>
		/// <param name="v1">Vector 1</param>
		/// <param name="v2">Vector 2</param>
		/// <param name="s">Scalar</param>
		private void VMad(ref System.Numerics.Vector3 dest, System.Numerics.Vector3 v1, System.Numerics.Vector3 v2, float s)
		{
			dest.X = v1.X + v2.X * s;
			dest.Y = v1.Y + v2.Y * s;
			dest.Z = v1.Z + v2.Z * s;
		}

		private bool GetSteerTarget(NavMeshQuery navMeshQuery, System.Numerics.Vector3 startPos, System.Numerics.Vector3 endPos, float minTargetDist, SharpNav.Pathfinding.Path path,
			ref System.Numerics.Vector3 steerPos, ref StraightPathFlags steerPosFlag, ref NavPolyId steerPosRef)
		{
			StraightPath steerPath = new StraightPath();
			navMeshQuery.FindStraightPath(startPos, endPos, path, steerPath, 0);
			int nsteerPath = steerPath.Count;
			if (nsteerPath == 0)
				return false;

			//find vertex far enough to steer to
			int ns = 0;
			while (ns < nsteerPath)
			{
				if ((steerPath[ns].Flags & StraightPathFlags.OffMeshConnection) != 0 ||
					!InRange(steerPath[ns].Point.Position, startPos, minTargetDist, 1000.0f))
					break;

				ns++;
			}

			//failed to find good point to steer to
			if (ns >= nsteerPath)
				return false;

			steerPos = steerPath[ns].Point.Position;
			steerPos.Y = startPos.Y;
			steerPosFlag = steerPath[ns].Flags;
			if (steerPosFlag == StraightPathFlags.None && ns == (nsteerPath - 1))
				steerPosFlag = StraightPathFlags.End; // otherwise seeks path infinitely!!!
			steerPosRef = steerPath[ns].Point.Polygon;

			return true;
		}

		private bool InRange(System.Numerics.Vector3 v1, System.Numerics.Vector3 v2, float r, float h)
		{
			float dx = v2.X - v1.X;
			float dy = v2.Y - v1.Y;
			float dz = v2.Z - v1.Z;
			return (dx * dx + dz * dz) < (r * r) && Math.Abs(dy) < h;
		}

		private void GenerateCrowd()
		{
			if (!hasGenerated || navMeshQuery == null)
			{
				return;
			}

			Console.WriteLine("Gen Crowd");

			Random rand = new Random();
			crowd = new Crowd(MAX_AGENTS, 0.6f, ref tiledNavMesh);
	
			var c = new System.Numerics.Vector3(10, 0, 0);
			var e = new System.Numerics.Vector3(5, 5, 5);

			AgentParams ap = new AgentParams();
			ap.Radius = 0.6f;
			ap.Height = 2.0f;
			ap.MaxAcceleration = 8.0f;
			ap.MaxSpeed = 3.5f;
			ap.CollisionQueryRange = ap.Radius * 12.0f;
			ap.PathOptimizationRange = ap.Radius * 30.0f;
			ap.UpdateFlags = new UpdateFlags();

			//initialize starting positions for each active agent
			for (int i = 0; i < numActiveAgents; i++)
			{
				//Get the polygon that the starting point is in
				NavPoint startPt;
				navMeshQuery.FindNearestPoly(ref c, ref e, out startPt);

				//Pick a new random point that is within a certain radius of the current point
				NavPoint newPt;
				navMeshQuery.FindRandomPointAroundCircle(ref startPt, 1000, out newPt);

				c = newPt.Position;

				//Save this random point as the starting position
				trails[i].Trail = new System.Numerics.Vector3[AGENT_MAX_TRAIL];
				trails[i].Trail[0] = newPt.Position;
				trails[i].HTrail = 0;

				//add this agent to the crowd
				int idx = crowd.AddAgent(newPt.Position, ap);

				//Give this agent a target point
				NavPoint targetPt;
				navMeshQuery.FindRandomPointAroundCircle(ref newPt, 1000, out targetPt);

				crowd.GetAgent(idx).RequestMoveTarget(targetPt.Polygon, targetPt.Position);
				trails[i].Trail[AGENT_MAX_TRAIL - 1] = targetPt.Position;
			}
		}
	}
}
