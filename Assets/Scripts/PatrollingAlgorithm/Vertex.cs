using System.Collections.Generic;
using UnityEngine;

class Vertex
{
    private List<Vertex> _neighbors;
    private float _weight;
    private int _idleness;
    private Vector2Int _position;
    private int _totalIdleness;
    private int _visits;
    private float _maxIdleness;

    public Vertex(float weight, Vector2Int position)
    {
        _weight = weight;
        _position = position;
    }

    public IReadOnlyList<Vertex> Neighbors
    {
        get { return _neighbors; }
    }

    public float Weight
    {
        get { return _weight; }
    }

    public int Idleness
    {
        get { return _idleness; }
    }

    public Vector2Int Position
    {
        get { return _position; }
    }

    public float AverageIdleness
    {
        get { return (float)_totalIdleness / (float)_visits; }
    }

    public void ResetIdleness()
    {
        _visits++;
        _totalIdleness += _idleness;

        if (_idleness > _maxIdleness)
        {
            _maxIdleness = _idleness;
        }
        _idleness = 0;
    }

    public void AddNeighbor(Vertex neighbor){
        _neighbors.Add(neighbor);
    }

    public void UpdateIdleness(){
        _idleness++;
    }
}
