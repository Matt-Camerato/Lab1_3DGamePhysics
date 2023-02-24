using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Neighborhood : MonoBehaviour
{
    [Header("Set Dynamically")]
    public List<Boid>       neighbors;
    public List<Obstacle>   obstacles;
    private SphereCollider  coll;

    void Start()
    {
        neighbors = new List<Boid>();
        coll = GetComponent<SphereCollider>();
        coll.radius = Spawner.S.neighborDist / 2;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (coll.radius != Spawner.S.neighborDist/2)
        {
            coll.radius = Spawner.S.neighborDist/2;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        Boid b = other.GetComponent<Boid>();
        if (b != null)
        {
            if (neighbors.IndexOf(b) == -1)
            { 
                neighbors.Add(b);
            }
        }
        
        //add obstacles to list if within range
        Obstacle o = other.GetComponent<Obstacle>();
        if (o != null)
        {
            if (obstacles.IndexOf(o) == -1)
            { 
                obstacles.Add(o);
            }
        }
    }

    void OnTriggerExit(Collider other)
    {
        Boid b = other.GetComponent<Boid>();
        if (b != null)
        {
            if (neighbors.IndexOf(b) != -1)
            {
                neighbors.Remove(b);
            }
        }

        //remove obstacles from list when out of range
        Obstacle o = other.GetComponent<Obstacle>();
        if (o != null)
        {
            if (obstacles.IndexOf(o) != -1)
            { 
                obstacles.Remove(o);
            }
        }
    }


    public Vector3 avgPos
    {
        get
        {
            Vector3 avg = Vector3.zero;
            if (neighbors.Count == 0) return avg;

            for (int i=0; i<neighbors.Count; i++)
            {
                avg += neighbors[i].pos;
            }
            avg /= neighbors.Count;

            return avg;

        }
    }

    //average YPos for flock remaining level
    public float avgYPos
    {
        get
        {
            float avg = 0;
            if (neighbors.Count == 0) return avg;

            for (int i=0; i<neighbors.Count; i++)
            {
                avg += neighbors[i].pos.y;
            }
            avg /= neighbors.Count;

            return avg;
        }
    }

    public Vector3 avgVel
    {
        get
        {
            Vector3 avg = Vector3.zero;
            if (neighbors.Count == 0) return avg;

            for (int i = 0; i < neighbors.Count; i++)
            {
                avg += neighbors[i].rb.velocity;
            }
            avg /= neighbors.Count;

            return avg;
        }
    }

    public Vector3 avgClosePos
    {
        get
        {
            Vector3 avg = Vector3.zero;
            Vector3 delta;
            int nearCount = 0;
            for (int i = 0; i < neighbors.Count; i++)
            {
                delta = neighbors[i].pos - transform.position;
                if (delta.magnitude <= Spawner.S.collDist)
                {
                    avg += neighbors[i].pos;
                    nearCount++;
                }
            }

            // If there were no neighbors too close, return Vector3.zero
            if (nearCount == 0) return avg;

            // Otherwise, averge their locations
            avg /= nearCount;
            return avg;
        }
    }

    public Vector3 avgCloseObstaclePos
    {
        get
        {
            Vector3 avg = Vector3.zero;
            Vector3 delta;
            int nearCount = 0;
            for (int i = 0; i < obstacles.Count; i++)
            {
                delta = obstacles[i].transform.position - transform.position;
                if (delta.magnitude <= Spawner.S.collDist)
                {
                    avg += obstacles[i].transform.position;
                    nearCount++;
                }
            }

            // If there were no neighbors too close, return Vector3.zero
            if (nearCount == 0) return avg;

            // Otherwise, averge their locations
            avg /= nearCount;
            return avg;
        }
    }
}
