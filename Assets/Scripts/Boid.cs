using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    [Header("Set Dynamically")]
    public Rigidbody rb;
    public SphereCollider sphereCollider;
    private Neighborhood neighborhood;
    
    public Vector3 vecToAtt;
    public Vector3 vecToAtt2;
    public Vector3 attPos;

    static Material lineMaterial;
    public float spinRate = 0.50f;
    public Quaternion myq;
    //public Vector3 boidRotVec;
    public float dangle;
    public Vector3 rotVec2;
    public float myangle;
    private float rotationRate = 0.50f;

    public bool fastBoid; //bool to determine type of boid (green is fast, blue is slow)

    public float propAngle;
    public Quaternion propQ;
    public Vector3 propVec2;
    

    public Vector3 pos
    {
        get { return transform.position; }
        set { transform.position = value; }
    }

    public Quaternion boidRot
    {
        get { return transform.rotation; }
        set { transform.rotation = value; }
    }

    private void Awake()
    {
        //initialize component fields
        rb = GetComponent<Rigidbody>();
        sphereCollider = GetComponent<SphereCollider>();
        neighborhood = GetComponent<Neighborhood>();

        //boidRotVec = Vector3.up;
        dangle = 0.0f;
        rotVec2 = Vector3.zero;
        myangle = 0.0f;
        propAngle = 0.0f;

        //Set a random initial position
        pos = Random.insideUnitSphere * Spawner.S.spawnRadius;

        //determine type of boid
        if(Random.value > 0.5f) fastBoid = true;
        else fastBoid = false;

        //Set a random initial velocity (considering type of boid)
        float boidVel = fastBoid? Spawner.S.velocity * 2 : Spawner.S.velocity / 2;
        Vector3 vel = Random.onUnitSphere * boidVel;
        rb.velocity = vel;

        //Construct the unit vector 15 deg shifted from the 'ray' pointing from boid to attractor
        vecToAtt = Attractor.POS - pos; //calculate vector from boid to attractor
        rotVec2 = vecToAtt.normalized * sphereCollider.radius; //resize to collider radius
        Vector3 myPerp = Vector3.Cross(Vector3.up, vecToAtt); //find a perpendicular to vecToAtt
        myq = Quaternion.AngleAxis(15, myPerp); //construct quaternion rotation for 15 deg around myPerp
        rotVec2 = myq * rotVec2; //rotate rotVec2. this is the vector we will spin about 'ray'


        Vector3 forceVec = rb.velocity; //get force vector from rigidbody
        propVec2 = forceVec.normalized * sphereCollider.radius; //create vector that will spin around force vector
        Vector3 forcePerp = Vector3.Cross(Vector3.up, forceVec); //find a perpendicular to force vector
        propQ = Quaternion.AngleAxis(90, forcePerp);
        propVec2 = propQ * propVec2;

        //Color boids based on the type of boid they are
        Color boidColor;
        if(fastBoid)
        {
            boidColor = new Color(0.5f, 1f, 0.5f); //fast boids are green
        }
        else
        {
            boidColor = new Color(0.55f, 0.8f, 1f); //slow boids are blue
        }
        Renderer[] rends = gameObject.GetComponentsInChildren<Renderer>();
        foreach (Renderer r in rends)
        {
            r.material.color = boidColor;
        }
        rends[3].material.color = Color.white; //make propellor white
        TrailRenderer tRend = GetComponent<TrailRenderer>();
        tRend.material.SetColor("_TintColor", boidColor);
    }

    private void FixedUpdate()
    {
        Vector3 vel = rb.velocity;
        Spawner spawner = Spawner.S;
        float spawnerVel = fastBoid? spawner.velocity * 2 : spawner.velocity / 2;

        //Collision Avoidance - avoid neigbors who are too close
        Vector3 velAvoid = Vector3.zero;
        Vector3 tooClosePos = neighborhood.avgClosePos;
        if (tooClosePos != Vector3.zero) //if the response is Vector3.zero, then no need to react
        {
            velAvoid = pos - tooClosePos;
            velAvoid.Normalize();
            velAvoid *= spawnerVel;
        }

        //Obstacle Avoidance - avoid obstacles that are too close
        Vector3 obstacleAvoid = Vector3.zero;
        Vector3 obstacleTooClosePos = neighborhood.avgCloseObstaclePos;
        if (obstacleTooClosePos != Vector3.zero) //if the response is Vector3.zero, then no need to react
        {
            obstacleAvoid = pos - obstacleTooClosePos;
            obstacleAvoid.Normalize();
            obstacleAvoid *= spawnerVel;
        }

        //Velocity matching - Try to match velocity with neigbors
        Vector3 velAlign = neighborhood.avgVel;
        if (velAlign != Vector3.zero) //only do more if the velAlign is not Vector3.zero
        {
            velAlign.Normalize(); //just need direction, so normalize velocity
            velAlign *= spawnerVel; //then set it to the speed we chose
        }


        //Flock centering - move towards the center of local neighbors 
        Vector3 velCenter = neighborhood.avgPos;
        if (velCenter != Vector3.zero)
        {
            velCenter -= transform.position;
            velCenter.Normalize();
            velCenter *= spawnerVel;
        }

        //Flock centering out on Y-axis
        float yCenter = neighborhood.avgYPos;
        Vector3 yAlign = Vector3.zero;
        if(yCenter != 0)
        {
            yCenter -= transform.position.y;
            yAlign = Vector3.up * yCenter;
            yAlign.Normalize();
            yAlign *= spawnerVel;
        }

        //ATTRACTION - Move towards the Attractor
        Vector3 delta = Attractor.POS - pos;
        //check whether we're attracted or avoiding the Attractor
        bool attracted = (delta.magnitude > spawner.attractPushDist);
        Vector3 velAttract = delta.normalized * spawnerVel;

        //Apply all the velocities
        float fdt = Time.fixedDeltaTime;
        if (velAvoid != Vector3.zero) vel = Vector3.Lerp(vel, velAvoid, spawner.collAvoid);
        else if(obstacleAvoid != Vector3.zero) vel = Vector3.Lerp(vel, obstacleAvoid, spawner.collAvoid);
        else
        {
            if (velAlign != Vector3.zero)
            {
                vel = Vector3.Lerp(vel, velAlign, spawner.velMatching * fdt);
            }
            if (velCenter != Vector3.zero)
            {
                vel = Vector3.Lerp(vel, velAlign, spawner.flockCentering * fdt);
            }
            if(yAlign != Vector3.zero)
            {
                vel = Vector3.Lerp(vel, yAlign, spawner.yCenterning * fdt);
            }
            if (velAttract != Vector3.zero)
            {
                if (attracted)
                {
                    vel = Vector3.Lerp(vel, velAttract, spawner.attractPull * fdt);
                }
                else
                {
                    vel = Vector3.Lerp(vel, -velAttract, spawner.attractPush * fdt);
                }
            }
        }

        //set vel to the velocity set on the spawner singleton
        vel = vel.normalized * spawnerVel;
        rb.velocity = vel; //finally assign this to the Rigidbody

        //Update the line pointing to the attractor with spherecollider radius
        //Get attractor postion and draw it
        attPos = Attractor.POS;
        vecToAtt = attPos - pos;
        //normalize vectoatt and multiply by collider radius to change vector length
        vecToAtt = vecToAtt.normalized * sphereCollider.radius;
        //Now the line from pos to pos + vecToAtt that is drawn with GL.Lines below
        // is the vector from the boid pointing towards the attractor with the radius of
        // the SphereCollider of the boid

        //Now draw a line projecting vecToAtt on the plane:
        //this is the vector from boid to attractor again
        //this vector with the y-component equal to zero lies in the plane x,z
        //it is plotted below between pos and attPos by setting y-components to zero

        //(By the way this only works if the plane is the x,z plane!
        //To generalize, we would compute the normal to the plane say nplane
        // then the vectors 
        //vecToAtt2Perp = Vector3.Dot(nplane,vecToAtt2)*nplane
        //posPerp = Vector3.Dot(nplane,pos)*nplan
        //are the ammounts of the pos and vecToAtt2 that are perpendicular to the plane
        // Now subtract 
        // vecToAtt2InPlane = vecToAtt2 - vecToAtt2Perp;
        // posInPlane = pos - posPerp;
        // ploting a line between these two gives the projectioin of vecToAtt2
        // in the general plane)

        //Now spin each boid about the vector it is moving in

        //create the quaternion and rotate boid via transform.rotation
        dangle += spinRate * Time.deltaTime;
        boidRot = Quaternion.AngleAxis(dangle, vecToAtt);

        //Now update, rotate and draw the vector 15 deg from the boids 'ray' to the attractor
        float delta_myangle = rotationRate * Time.deltaTime; //update the angle step
        myangle += delta_myangle; //add the angle step to the angle
        myq = Quaternion.AngleAxis(myangle, vecToAtt); //update the quaternion rotation
        //Debug.Log("myq " + myq.w + " " + myq.x + " " + myq.y + " " + myq.z);
        rotVec2 = myq * rotVec2; //rotate the vector on the 15 deg cone

        
        
        RotatePropellor();
        LookAhead();
    }

    //orients propellor to look in the direction its flying
    private void RotatePropellor()
    {
        transform.GetChild(2).rotation *= propQ;

        //increase angle over time
        float dPropAngle = rotationRate * Time.deltaTime;
        propAngle += dPropAngle;

        Vector3 forceVec = transform.GetChild(2).position + rb.velocity; //get force vector from rigidbody
        propQ = Quaternion.AngleAxis(propAngle, forceVec); //rotate propellor around force vector
        propVec2 = propQ * propVec2;
    }

    //orients the Boid to look at the direction it's flying
    private void LookAhead()
    {
        transform.LookAt(pos + rb.velocity);
        Vector3 tvec;
        tvec = boidRot * Vector3.up;
        Quaternion qtemp = Quaternion.LookRotation(pos + rb.velocity, tvec);
        //transform.rotation = qtemp;
        //transform.LookAt(pos + rb.velocity,tvec);
        //transform.rotation = boidRot;
        //Debug.Log("myq " + myq.w + " " + myq.x + " " + myq.y + " " + myq.z);
    }

    public void OnRenderObject()
    {
        // Draw lines
        CreateLineMaterial();
        // Apply the line material
        lineMaterial.SetPass(0);
        GL.PushMatrix();
        // Set transformation matrix for drawing to
        // match our transform
        //GL.MultMatrix(transform.localToWorldMatrix);

        GL.Begin(GL.LINES);

        //this is the line pointing from boid to attractor
        /*
        Color color1 = Color.red;
        GL.Color(color1);        
        GL.Vertex(pos); //boid position
        //GL.Vertex(attPos); // this line attractor position
        GL.Vertex(pos + vecToAtt); //position of end of vector pointing from boid to attractor
        */

        //this is the component of the line pointing from boid to attractor that lies in the x,z plane
        /*
        Color color2 = Color.blue;
        GL.Color(color2);
        GL.Vertex3(pos.x,0,pos.z);//x,z component of boid position
        GL.Vertex3(attPos.x,0,attPos.z);//x,z component of attractor position
        */

        /*
        //this is the line rotating on the 15 deg cone that is centered on attPos
        Color color3 = Color.green;
        GL.Color(color3);
        GL.Vertex(pos);
        GL.Vertex(pos + rotVec2);
        */

        //this line represents the propellor axis (velocity vector from propellor tip)
        /*
        Color color4 = Color.blue;
        GL.Color(color4);
        GL.Vertex(transform.GetChild(2).position);
        GL.Vertex(transform.GetChild(2).position + rb.velocity);
        */

        //these lines represents the propellor blades
        /*
        Color color5 = Color.white;
        GL.Color(color5);
        GL.Vertex(transform.GetChild(2).position);
        GL.Vertex(transform.GetChild(2).position + propVec2);
        */
       

        GL.End();
        GL.PopMatrix();
    }

    private static void CreateLineMaterial()
    {
        if (!lineMaterial)
        {
            // Unity has a built-in shader that is useful for drawing
            // simple colored things.
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            // Turn on alpha blending
            lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn backface culling off
            lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
        }
    }
}