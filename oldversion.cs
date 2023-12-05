using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
//using static UnityEditor.FilePathAttribute;
using System;
using MathNet.Numerics.Statistics;
using MathNet.Numerics.LinearAlgebra.Factorization;
//using System.Numerics;
//using System.Diagnostics;

public class Tranform : MonoBehaviour
{
    public GameObject spawnee;
    public Material Pmaterial;
    public Material Qmaterial;


    void Start()
    {


        Matrix<double> A = CreateMatrix.DenseOfArray(new double[,] {
            { 1, 4, 7 },
            { 2, 5, 8 },
            { 3, 6, 9 }
        });

        Matrix<double> B = CreateMatrix.DenseOfArray(new double[,] {
            { 10, 14, 17},
            { 2, 5, 8 },
            { 3, 6, 9 }
        });
        Debug.Log("Original A:");
        Debug.Log(A);

        Debug.Log("\nOriginal (shifted by 10 unit) B:");
        Debug.Log(B);

        RigidTransform3D(A, B);




        // Applying a rotation of 45 degrees around the z-axis and a translation of (1, 2, 3)
        //Matrix<double> R = RotationMatrix(45, Axis.Z);
        //Matrix<double> t = CreateMatrix.DenseOfArray(new double[3,3] { { 1,1,1 }, { 2,2,2 }, { 3,3,3 } });

        //Matrix<double> B = R * A + t;


    }

    private void RigidTransform3D(Matrix<double> A, Matrix<double> B)
    {
        if (A.RowCount != 3 || B.RowCount != 3 || A.ColumnCount != B.ColumnCount)
            throw new ArgumentException("Input matrices must be 3xN");

        Vector<double> centroid_A = Vector<double>.Build.DenseOfArray(new[] { A.Column(0).Mean(), A.Column(1).Mean(), A.Column(2).Mean() });
        Vector<double> centroid_B = Vector<double>.Build.DenseOfArray(new[] { B.Column(0).Mean(), B.Column(1).Mean(), B.Column(2).Mean() });


        var p = Matrix<double>.Build.DenseOfColumns(new[] { A.Column(0) - centroid_A, A.Column(1) - centroid_A, A.Column(1) - centroid_A });
        var q = Matrix<double>.Build.DenseOfColumns(new[] { B.Column(0) - centroid_B, B.Column(1) - centroid_B, B.Column(1) - centroid_B });

        var H = p * q.Transpose();

        var svd = H.Svd(true);

        Matrix<double> U = svd.U;
        Vector<double> S = svd.S;
        Matrix<double> VT = svd.VT;

        Matrix<double> V = VT.Transpose();
        Matrix<double> R = V * U.Transpose();

        if (R.Determinant() < 0)
        {
            R.Svd(true);
            var V = R.VT;
        }



        var translation = centroid_B - R * centroid_A;
        var rotation = R;
    }

    Matrix<double> RotationMatrix(double angle, Axis axis)
    {
        double angleRad = (Math.PI / 180) * angle;

        double cos = Math.Cos(angleRad);
        double sin = Math.Sin(angleRad);

        Matrix<double> rotationMatrix = Matrix<double>.Build.DenseIdentity(3);

        switch (axis)
        {
            case Axis.X:
                rotationMatrix[1, 1] = cos;
                rotationMatrix[1, 2] = -sin;
                rotationMatrix[2, 1] = sin;
                rotationMatrix[2, 2] = cos;
                break;

            case Axis.Y:
                rotationMatrix[0, 0] = cos;
                rotationMatrix[0, 2] = sin;
                rotationMatrix[2, 0] = -sin;
                rotationMatrix[2, 2] = cos;
                break;

            case Axis.Z:
                rotationMatrix[0, 0] = cos;
                rotationMatrix[0, 1] = -sin;
                rotationMatrix[1, 0] = sin;
                rotationMatrix[1, 1] = cos;
                break;
        }

        return rotationMatrix;
    }

    public enum Axis
    {
        X,
        Y,
        Z
    }







    //void SpawnPoints(Vector3[] P, Vector3[] Q)
    //{
    //    Renderer spawneeRenderer = spawnee.GetComponent<Renderer>();
    //    spawneeRenderer.material = Pmaterial;
    //    for (int i = 0; i < 3; i++)
    //        Instantiate(spawnee, Q[i], Quaternion.identity);

    //    spawneeRenderer.material = Qmaterial;

    //    for (int i = 0; i < 3; i++)
    //        Instantiate(spawnee, P[i], Quaternion.identity);

    //}


    //void RANSAC(Vector3[] P, Vector3[] Q)
    //{
    //    int maxIterations = 30;
    //    //float inlierThreshold = 0.1f; 

    //    //int bestInlierCount = 0;
    //    //Quaternion bestRotation = Quaternion.identity;
    //    //Vector3 bestTranslation = Vector3.zero;

    //    for (int iteration = 0; iteration < maxIterations; iteration++)
    //    {
    //        //int[] randomIndices = GenerateRandomIndices(Q.Length, 3);
    //        Vector3[] randomPointsP = { P[0], P[1], P[2] };
    //        Vector3[] randomPointsQ = { Q[0], Q[1], Q[2] };

    //        // Calculate transformation between the random points
    //        //Quaternion rotation;
    //        //Vector3 translation;
    //        ComputeRigidTransform(randomPointsP, randomPointsQ);




    //        // // Apply transformation to the entire set of points in Q
    //        // Vector3[] transformedQ = ApplyTransformation(Q, rotation, translation);

    //        // // Count inliers (points in Q close to their corresponding points in P)
    //        // int inlierCount = CountInliers(transformedQ, P, inlierThreshold);

    //        // // Update best transformation if current has more inliers
    //        // if (inlierCount > bestInlierCount)
    //        // {
    //        //     bestInlierCount = inlierCount;
    //        //     bestRotation = rotation;
    //        //     bestTranslation = translation;
    //        // }
    //    }

    //}


    //int[] GenerateRandomIndices(int maxIndex, int count)
    //{
    //    // Generate 'count' unique random indices between 0 and maxIndex
    //    int[] indices = new int[count];
    //    for (int i = 0; i < count; i++)
    //    {
    //        int randomIndex;
    //        randomIndex = Random.Range(0, maxIndex);

    //        indices[i] = randomIndex;
    //    }

    //    return indices;
    //}


    //void ComputeRigidTransform(Vector3[] P, Vector3[] Q, out Quaternion rotation, out Vector3 translation)
    //{
    //    Vector3 centroidP = ComputeCentroid(P);
    //    Vector3 centroidQ = ComputeCentroid(Q);

    //    Matrix4x4 H = ComputeCovarianceMatrix(P, Q, centroidP, centroidQ);



    //    var denseMatrix = DenseMatrix.OfArray(new double[,]
    //    {
    //        { H[0, 0], H[0, 1], H[0, 2], H[0, 3] },
    //        { H[1, 0], H[1, 1], H[1, 2], H[1, 3] },
    //        { H[2, 0], H[2, 1], H[2, 2], H[2, 3] },
    //        { H[3, 0], H[3, 1], H[3, 2], H[3, 3] }
    //    });

    //    MatrixBuilder<double> PCentroid = Matrix.Build;

    //    Vector<double> centroid;
    //    //var V = Vector<double>.Build;

    //    //var v = V.Dense(10);



    //    var svd = denseMatrix.Svd(true);

    //    Matrix<double> U = svd.U;
    //    Matrix<double> VT = svd.VT;

    //    Matrix<double> V = VT.Transpose();
    //    Matrix<double> R = V * U.Transpose();

    //    var rotationArray = R.ToArray(); ;


    //    translation = centroidQ - R * centroidP;


    //}

    //Vector3 ComputeCentroid(Vector3[] points)
    //{
    //    Vector3 centroid = Vector3.zero;
    //    foreach (Vector3 point in points)
    //    {
    //        centroid += point;
    //    }
    //    return centroid / points.Length;
    //}

    //Matrix4x4 ComputeCovarianceMatrix(Vector3[] P, Vector3[] Q, Vector3 centroidP, Vector3 centroidQ)
    //{
    //    Matrix4x4 H = Matrix4x4.zero;

    //    for (int i = 0; i < P.Length; i++)
    //    {
    //        Vector3 p = P[i] - centroidP;
    //        Vector3 q = Q[i] - centroidQ;

    //        H[0, 0] += p.x * q.x;
    //        H[0, 1] += p.x * q.y;
    //        H[0, 2] += p.x * q.z;
    //        H[1, 0] += p.y * q.x;
    //        H[1, 1] += p.y * q.y;
    //        H[1, 2] += p.y * q.z;
    //        H[2, 0] += p.z * q.x;
    //        H[2, 1] += p.z * q.y;
    //        H[2, 2] += p.z * q.z;
    //    }

    //    return H;
    //}




}






