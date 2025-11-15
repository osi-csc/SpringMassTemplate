using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring (think about this ... may not even be needed o_0
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // is thi sparticle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = false;            // To render or not to render

     /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/
    // Public inspector variables
    public bool handlePlaneCollisions = true;
    public float particleMass = 1.0f;
    public bool useGravity = true;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);

    
    private Mesh mesh;
    private BParticle[] particles;
    private BPlane plane;
    private GameObject groundPlaneObject; // Reference to the actual ground plane

    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
 
    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/

    void InitParticles()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        
        particles = new BParticle[vertices.Length];
        
        //initialize particles from mesh vertices
        for (int i = 0; i < vertices.Length; i++)
        {
            particles[i] = new BParticle
            {
                position = transform.TransformPoint(vertices[i]),
                velocity = Vector3.zero,
                mass = particleMass,
                attachedToContact = false,
                attachedSprings = new List<BSpring>(),
                currentForces = Vector3.zero,
                contactSpring = new BContactSpring()
            };
        }
        
        //create springs
        for (int i = 0; i < particles.Length; i++)
        {
            for (int j = i + 1; j < particles.Length; j++)
            {
                float restLength = Vector3.Distance(particles[i].position, particles[j].position);
                
                BSpring spring = new BSpring
                {
                    ks = defaultSpringKS,
                    kd = defaultSpringKD,
                    restLength = restLength,
                    attachedParticle = j
                };
                
                particles[i].attachedSprings.Add(spring);
            }
        }
    }

    void InitPlane()
    {
        //find ground plane
        groundPlaneObject = GameObject.Find("GroundPlane"); 
  
        plane.position = groundPlaneObject.transform.position;
        plane.normal = groundPlaneObject.transform.up; 

    }

    void UpdateMesh()
    {
        Vector3[] vertices = new Vector3[particles.Length];
        
        for (int i = 0; i < particles.Length; i++)
        {
            //convert world position to local coordinates for mesh
            vertices[i] = transform.InverseTransformPoint(particles[i].position);
        }
        
        mesh.vertices = vertices;
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }

    void ResetParticleForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].currentForces = Vector3.zero;
        }
    }

//everything after is for updating
    void ApplyGravity()
    {
        if (!useGravity) return; //ignore if gravity is not applied
        
        for (int i = 0; i < particles.Length; i++) //else loop
        {
            particles[i].currentForces += gravity * particles[i].mass;
        }
    }

    void ApplySpringForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            foreach (BSpring spring in particles[i].attachedSprings)
            {
                int j = spring.attachedParticle;
                Vector3 deltaPos = particles[i].position - particles[j].position;
                float distance = deltaPos.magnitude;
                
                if (distance > 0.01f)
                {
                    Vector3 direction = deltaPos / distance;
                    Vector3 deltaVel = particles[i].velocity - particles[j].velocity;
                    
                    // Spring force: k_s * (l - |x_i - x_j|) * direction
                    float springForceMagnitude = spring.ks * (spring.restLength - distance);
                    Vector3 springForce = springForceMagnitude * direction;
                    
                    // Damping force: k_d * ((v_i - v_j) · direction) * direction
                    float dampingForceMagnitude = spring.kd * Vector3.Dot(deltaVel, direction);
                    Vector3 dampingForce = dampingForceMagnitude * direction;
                    
                    // Total force and apply to both particles (reflected trick)
                    Vector3 totalForce = springForce - dampingForce;
                    particles[i].currentForces += totalForce;
                    particles[j].currentForces -= totalForce;
                }
            }
        }
    }

    void HandleGroundCollisions()
    {
        if (!handlePlaneCollisions || groundPlaneObject == null) return;
        
        for (int i = 0; i < particles.Length; i++)
        {
            //calc distance to plane
            Vector3 toParticle = particles[i].position - plane.position;
            float distanceToPlane = Vector3.Dot(toParticle, plane.normal);
            
            if (distanceToPlane < 0) //below plane 
            {
                if (!particles[i].attachedToContact) //if new collision
                {
                    //Init contact spring, attach point is the nearest point on the plane
                    Vector3 nearestPoint = particles[i].position - distanceToPlane * plane.normal;
                    particles[i].contactSpring = new BContactSpring
                    {
                        ks = contactSpringKS,
                        kd = contactSpringKD,
                        restLength = 0f, //particle on plane surface
                        attachPoint = nearestPoint
                    };
                    particles[i].attachedToContact = true;
                }
                
                //penalty force: -k_s((x_p - x_g) · n)n - k_d * v_p
                Vector3 penetrationVector = particles[i].position - particles[i].contactSpring.attachPoint;
                float penetrationDepth = Vector3.Dot(penetrationVector, plane.normal);
                
                Vector3 springForce = -particles[i].contactSpring.ks * penetrationDepth * plane.normal;
                Vector3 dampingForce = -particles[i].contactSpring.kd * particles[i].velocity;
                
                particles[i].currentForces += springForce + dampingForce;
            }
            else
            {
                //Particle is above the plane, no contact
                particles[i].attachedToContact = false;
            }
        }
    }

    void SymplecticEulerIntegration(float deltaTime)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            // a = F/m
            Vector3 acceleration = particles[i].currentForces / particles[i].mass;
            
            // v = v + a * dt
            particles[i].velocity += acceleration * deltaTime;
            
            // x = x + v * dt
            particles[i].position += particles[i].velocity * deltaTime;
        }
    }
    void Start()
    {
        InitParticles();
        InitPlane();
    }

    void FixedUpdate() //update is too fast so using fixed update
    {
        float deltaTime = Time.fixedDeltaTime;
        
        ResetParticleForces();
        ApplyGravity();
        ApplySpringForces();
        HandleGroundCollisions();
        SymplecticEulerIntegration(deltaTime);
        UpdateMesh();
        
        // Debug rendering
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                // Draw force vectors in blue
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces * 0.1f, Color.blue);

                // Draw springs in red
                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
                
                // Draw contact springs in green if active
                if (particles[i].attachedToContact)
                {
                    Debug.DrawLine(particles[i].position, particles[i].contactSpring.attachPoint, Color.green);
                }
            }
            
            // Also draw the ground plane normal for reference
            Debug.DrawLine(plane.position, plane.position + plane.normal * 2f, Color.yellow);
        }
    }

}