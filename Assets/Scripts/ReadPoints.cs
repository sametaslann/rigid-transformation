using Unity.VisualScripting;
using UnityEngine;

namespace Assets.Scripts
{
    public class ReadPoints: MonoBehaviour
    {
        [SerializeField] string fileName;
        [SerializeField] Material material;


        void Update()
        {
            
        }
        void Awake()
        {
            string path = Application.dataPath + "/PointClouds/" + fileName + ".txt";
            string[] lines = System.IO.File.ReadAllLines(path);

            int n = int.Parse(lines[0]);

            for (int i = 1; i <= n; i++)
            {
                string[] line = lines[i].Split(' ');
                float x = ToFloat(line[0]);
                float y = ToFloat(line[1]);
                float z = ToFloat(line[2]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.position = new Vector3(x, y, z);
                sphere.transform.parent = transform;



                sphere.GetComponent<Renderer>().material = material;
                sphere.name = "P " + i;

            }


            float ToFloat(string s)
            {
                return float.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
            }

        }
    }
}
