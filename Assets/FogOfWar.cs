using System.Collections;
using System.Collections.Generic;
using Maes;
using UnityEngine;

public class FogOfWarScript : MonoBehaviour
{

    public SimulationManager simulationManager;
    public GameObject _fogOfWarPlane;
    public List<Transform> robots;
    public LayerMask _foglayer;
    private float _revealRadius;
    private float _revealRadiusSqr;

    private Mesh _mesh;
    private Vector3[] _vertices;
    private Color[] _colors;

    // Use this for initialization
    void Start()
    {
        foreach (Maes.Robot.MonaRobot robot in simulationManager.CurrentSimulation.Robots)
        {
            robots.Add(robot.transform);
        }

        _revealRadius = simulationManager._currentScenario.RobotConstraints.SlamRayTraceRange;
        _revealRadiusSqr = _revealRadius * _revealRadius;
        _mesh = _fogOfWarPlane.GetComponent<MeshFilter>().mesh;
        _vertices = _mesh.vertices;
        _colors = new Color[_vertices.Length];
        for (int i = 0; i < _colors.Length; i++)
        {
            _colors[i] = Color.black;
        }
        for (int i = 0; i < _vertices.Length; i++)
        {
            _vertices[i] = _fogOfWarPlane.transform.TransformPoint(_vertices[i]);
        }
        UpdateColor();
    }

    // Update is called once per frame
    void Update()
    {
        foreach (Transform robot in robots)
        {
            Ray r = new Ray(transform.position, robot.position - transform.position);
            RaycastHit hit;
            if (Physics.Raycast(r, out hit, 1000, _foglayer, QueryTriggerInteraction.Collide))
            {
                for (int i = 0; i < _vertices.Length; i++)
                {
                    Vector3 v = _vertices[i];
                    float dist = Vector3.SqrMagnitude(v - hit.point);
                    if (dist < _revealRadiusSqr)
                    {
                        float alpha = Mathf.Min(_colors[i].a, dist / _revealRadiusSqr);
                        _colors[i].a = alpha;
                    }
                }
                UpdateColor();
            }
        }

    }

    void UpdateColor()
    {
        _mesh.colors = _colors;
    }
}