using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;
        public float ks;
        public float restLength;
        public int attachedParticle;
    }

    public struct BContactSpring
    {
        public float kd;
        public float ks;
        public float restLength;
        public Vector3 attachPoint;
    }

    public struct BParticle
    {
        public Vector3 position;
        public Vector3 velocity;
        public float mass;
        public BContactSpring contactSpring;
        public bool attachedToContact;
        public List<BSpring> attachedSprings;
        public Vector3 currentForces;
    }

    public struct BPlane
    {
        public Vector3 position;
        public Vector3 normal;
    }

    public float contactSpringKS = 1000.0f;
    public float contactSpringKD = 20.0f;
    public float defaultSpringKS = 100.0f;
    public float defaultSpringKD = 1.0f;
    public bool debugRender = false;

    // Public inspector variables
    public bool handlePlaneCollisions = true;
    public float particleMass = 1.0f;
    public bool useGravity = true;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);

    private Mesh mesh;
    private BParticle[] particles;
    private BPlane plane;
    private GameObject groundPlaneObject; // Reference to the actual ground plane

    void InitParticles()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        
        particles = new BParticle[vertices.Length];
        
        // Initialize particles from mesh vertices
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
        
        // Create springs between all particle pairs (no duplicates)
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
        // Find the ground plane GameObject in the scene
        groundPlaneObject = GameObject.Find("GroundPlane"); // Adjust name if different
        
        if (groundPlaneObject != null)
        {
            plane.position = groundPlaneObject.transform.position;
            plane.normal = groundPlaneObject.transform.up; // Assuming the plane's up direction is the normal
            
            Debug.Log($"Ground plane found at position: {plane.position} with normal: {plane.normal}");
        }
        else
        {
            // Fallback: create a default ground plane at y=0
            groundPlaneObject = new GameObject("DefaultGroundPlane");
            plane.position = Vector3.zero;
            plane.normal = Vector3.up;
            Debug.LogWarning("No ground plane found in scene! Using default ground plane at y=0");
        }
    }

    void UpdateMesh()
    {
        Vector3[] vertices = new Vector3[particles.Length];
        
        for (int i = 0; i < particles.Length; i++)
        {
            // Convert world position back to local coordinates for the mesh
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

    void ApplyGravity()
    {
        if (!useGravity) return;
        
        for (int i = 0; i < particles.Length; i++)
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
                
                if (distance > 0.0001f) // Avoid division by zero
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
            // Calculate signed distance to plane
            Vector3 toParticle = particles[i].position - plane.position;
            float distanceToPlane = Vector3.Dot(toParticle, plane.normal);
            
            if (distanceToPlane < 0) // Particle is below the plane (penetrating)
            {
                if (!particles[i].attachedToContact)
                {
                    // Initialize contact spring - attach point is the nearest point on the plane
                    Vector3 nearestPoint = particles[i].position - distanceToPlane * plane.normal;
                    particles[i].contactSpring = new BContactSpring
                    {
                        ks = contactSpringKS,
                        kd = contactSpringKD,
                        restLength = 0f, // We want the particle to be at the plane surface
                        attachPoint = nearestPoint
                    };
                    particles[i].attachedToContact = true;
                }
                
                // Apply penalty force: -k_s((x_p - x_g) · n)n - k_d * v_p
                Vector3 penetrationVector = particles[i].position - particles[i].contactSpring.attachPoint;
                float penetrationDepth = Vector3.Dot(penetrationVector, plane.normal);
                
                Vector3 springForce = -particles[i].contactSpring.ks * penetrationDepth * plane.normal;
                Vector3 dampingForce = -particles[i].contactSpring.kd * particles[i].velocity;
                
                particles[i].currentForces += springForce + dampingForce;
            }
            else
            {
                // Particle is above the plane, no contact
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

    void FixedUpdate()
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

    void Start()
    {
        InitParticles();
        InitPlane();
    }

    void Update()
    {
        // Empty - all physics in FixedUpdate
    }
}