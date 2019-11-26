
using Newtonsoft.Json;

using NUnit.Framework;
using SharpNav.Geometry;
using System.Numerics;

namespace SharpNavTests.Libraries
{
    [TestFixture]
	class JsonTest
	{
		[Test]
		public void JsonNet_Test()
		{
			// Simple test to make sure that the JSON.Net library is working
			Triangle3 triangle = new Triangle3(new Vector3(3), new Vector3(4), new Vector3(5));
			string output = JsonConvert.SerializeObject(triangle);
			
		}
	}
}
