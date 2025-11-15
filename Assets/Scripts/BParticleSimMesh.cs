using System.Collections;
using System.Collections.Generic;
using System.Linq.Expressions;
using UnityEngine;

// Check this out we can require components be on a game object!
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
    ***/
    public Transform groundPlaneTransform;  // the ground plane transform (Transform)
    public bool handlePlaneCollisions;      // handlePlaneCollisions flag (bool)
    public float particleMass = 1.0f;       // particle mass (float)
    public bool useGravity = true;          // useGravity flag (bool)
    public Vector3 gravity = new Vector3(0, -9.8f, 0); //gravity value (Vector3)

    /***
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/
    
    private Mesh mesh;                 // Mesh (Mesh)
    private BParticle[] particles;     // array of particles (BParticle[])
    private BPlane plane;              // the plane (BPlane)



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
    
    //InitParticles()
    void InitParticles()
    {
         MeshFilter mf = GetComponent<MeshFilter>();
    mesh = mf.mesh;

    Vector3[] verts = mesh.vertices;
    int vCount = verts.Length;

    particles = new BParticle[vCount];

    // Create particles
    for (int i = 0; i < vCount; i++)
    {
        BParticle p = new BParticle();
        p.position = transform.TransformPoint(verts[i]);   // convert to world-space
        p.velocity = Vector3.zero;
        p.mass = particleMass;
        p.contactSpring = new BContactSpring();
        p.attachedToContact = false;
        p.attachedSprings = new List<BSpring>();
        p.currentForces = Vector3.zero;

        particles[i] = p;
    }

    // Create springs between particle pairs (i < j avoids duplicates)
    for (int i = 0; i < vCount; i++)
    {
        for (int j = i + 1; j < vCount; j++)
        {
            BSpring spring = new BSpring();
            spring.ks = defaultSpringKS;
            spring.kd = defaultSpringKD;

            float restLen = Vector3.Distance(
                particles[i].position,
                particles[j].position
            );
            spring.restLength = restLen;
            spring.attachedParticle = j;

            particles[i].attachedSprings.Add(spring);
        }
    }
    }

    //InitPlane()
    void InitPlane()
    {
        plane = new BPlane();

        // Plane position and normal are taken from the provided transform
        plane.position = groundPlaneTransform.position;
        plane.normal   = groundPlaneTransform.up.normalized;
    }

    //UpdateMesh()
    void UpdateMesh()
    {
        if (mesh == null || particles == null) return;

        Vector3[] verts = mesh.vertices;

        for (int i = 0; i < particles.Length; i++)
        {
            // convert world-space particle positions back to local mesh space
            verts[i] = transform.InverseTransformPoint(particles[i].position);
        }

        mesh.vertices = verts;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    //ResetParticleForces()
    void ResetParticleForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].currentForces = Vector3.zero;
        }
    }

    void Start()
    {
        InitParticles();
        InitPlane();
    }

    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
         if (particles == null) return;

        float dt = Time.fixedDeltaTime;

        ResetParticleForces();

        if (useGravity)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].currentForces += gravity * particles[i].mass;
            }
        }

        for (int i = 0; i < particles.Length; i++)
    {
        foreach (BSpring s in particles[i].attachedSprings)
        {
            int j = s.attachedParticle;

            Vector3 xi = particles[i].position;
            Vector3 xj = particles[j].position;
            Vector3 vi = particles[i].velocity;
            Vector3 vj = particles[j].velocity;

            Vector3 dir = xj - xi;
            float dist = dir.magnitude;
            if (dist == 0) continue;
            Vector3 n = dir / dist;

            // Hooke spring force
            float fSpring = s.ks * (dist - s.restLength);

            // Damping force
            float fDamp = s.kd * Vector3.Dot((vi - vj), n);

            Vector3 F = (fSpring + fDamp) * n;

            // Apply equal-opposite forces
            particles[i].currentForces += F;
            particles[j].currentForces -= F;
        }
    }

    // 4. Ground-plane collision springs
    if (handlePlaneCollisions)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 p = particles[i].position;
            Vector3 v = particles[i].velocity;

            float d = Vector3.Dot(p - plane.position, plane.normal);

            if (d < 0.0f)   // penetrating
            {
                // If newly attached, store contact point
                if (!particles[i].attachedToContact)
                {
                    BContactSpring cs = new BContactSpring();
                    cs.ks = contactSpringKS;
                    cs.kd = contactSpringKD;
                    cs.attachPoint = p - d * plane.normal;    // nearest plane point
                    particles[i].contactSpring = cs;
                    particles[i].attachedToContact = true;
                }

                // Apply penalty spring
                BContactSpring s = particles[i].contactSpring;

                Vector3 xg = s.attachPoint;
                Vector3 n = plane.normal;

                float penetration = Vector3.Dot((p - xg), n);

                Vector3 F =
                    -s.ks * penetration * n
                    -s.kd * v;

                particles[i].currentForces += F;
            }
            else
            {
                particles[i].attachedToContact = false;
            }
        }
    }

    // 5. Integrate motion (semi-implicit Euler)
    for (int i = 0; i < particles.Length; i++)
    {
        Vector3 a = particles[i].currentForces / particles[i].mass;
        particles[i].velocity += a * dt;
        particles[i].position += particles[i].velocity * dt;
    }

    // 6. Update mesh
    UpdateMesh();
        /* This will work if you have a correctly made particles array
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces, Color.blue);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
            }
        }
        */
    }
}
