// Copyright (c) 2013-2014 Robert Rouhani <robert.rouhani@gmail.com> and other contributors (see CONTRIBUTORS file).
// Licensed under the MIT License - https://raw.github.com/Robmaister/SharpNav/master/LICENSE

using System;


using SharpNav;
using SharpNav.Geometry;

//Doesn't compile if in an unsupported configuration

namespace SharpNav.Examples
{
	public partial class ExampleWindow
	{
		private NavMeshGenerationSettings settings;
		private AreaIdGenerationSettings areaSettings;


		private void InitializeUI()
		{
			settings = NavMeshGenerationSettings.Default;
			areaSettings = new AreaIdGenerationSettings();
			
		}

		private class AreaIdGenerationSettings
		{
			public float MaxTriSlope { get; set; }
			public float MinLevelHeight { get; set; }
			public float MaxLevelHeight { get; set; }
		}
	}
}

