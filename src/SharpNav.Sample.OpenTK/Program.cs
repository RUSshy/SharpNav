// Copyright (c) 2013, 2015 Robert Rouhani <robert.rouhani@gmail.com> and other contributors (see CONTRIBUTORS file).
// Licensed under the MIT License - https://raw.github.com/Robmaister/SharpNav/master/LICENSE

using System;

namespace SharpNav.Examples
{
	class Program
	{
		[STAThread]
		static void Main(string[] args)
		{
			using (ExampleWindow ex = new ExampleWindow())
			{
				ex.Run();
			}
		}
	}
}
