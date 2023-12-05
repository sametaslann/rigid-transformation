using UnityEngine;
using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;


using Random = UnityEngine.Random;
using UnityEngine.UI;

public class Tranform : MonoBehaviour
{
    public GameObject spawnee;

    public GameObject pointCloud_P;
    public GameObject pointCloud_Q;
    public Material alignedPointsMaterial;

    private bool isScalable = false;

    private Vector<double>[] P;
    private Vector<double>[] Q;

    private GameObject[] alignedSpheres;

    private readonly int iteration = 10000;
    private readonly double thresh_hold = 0.5f;


    public void startTransformWithScale()
    {
        isScalable = true;
        startTransform();
    }

    public void startTransform()
    {

        P = new Vector<double>[pointCloud_P.transform.childCount];
        Q = new Vector<double>[pointCloud_Q.transform.childCount];

        for (int i = 0; i < pointCloud_P.transform.childCount; i++)
        {
            double x_p = Convert.ToDouble(pointCloud_P.transform.GetChild(i).position.x);
            double y_p = Convert.ToDouble(pointCloud_P.transform.GetChild(i).position.y);
            double z_p = Convert.ToDouble(pointCloud_P.transform.GetChild(i).position.z);
            P[i] = Vector<double>.Build.DenseOfArray(new[] { x_p, y_p, z_p });
        }

        for (int i = 0; i < pointCloud_Q.transform.childCount; i++)
        {
            double x_q = Convert.ToDouble(pointCloud_Q.transform.GetChild(i).position.x);
            double y_q = Convert.ToDouble(pointCloud_Q.transform.GetChild(i).position.y);
            double z_q = Convert.ToDouble(pointCloud_Q.transform.GetChild(i).position.z);
            Q[i] = Vector<double>.Build.DenseOfArray(new[] { x_q, y_q, z_q });
        }
        RANSAC();
    }


    private void RANSAC()
    {

        LinkedList<int>[] chosenIndices = new LinkedList<int>[P.Length];

        int best_result = -1;

        Matrix<double> best_rotation = Matrix<double>.Build.DenseIdentity(3);
        Vector<double> best_translation = Vector<double>.Build.Dense(3);
        double best_scale = 1;



        for (int i = 0; i < iteration; i++)
        {
            int index_P1 = Random.Range(0, P.Length);
            int index_P2 = Random.Range(0, P.Length);
            int index_P3 = Random.Range(0, P.Length);

            int index_Q1 = Random.Range(0, Q.Length);
            int index_Q2 = Random.Range(0, Q.Length);
            int index_Q3 = Random.Range(0, Q.Length);

            var points_from_P = Matrix<double>.Build.DenseOfColumns(new[] { P[index_P1], P[index_P2], P[index_P3] });
            var points_from_Q = Matrix<double>.Build.DenseOfColumns(new[] { Q[index_Q1], Q[index_Q2], Q[index_Q3] });

            Matrix<double> temp_rotation;
            Vector<double> temp_translation;
            double temp_scale;


            GetRigidTransformation(points_from_P, points_from_Q, out temp_rotation, out temp_translation, out temp_scale);


            int overlappings = CountOverlappings(temp_rotation, temp_translation, temp_scale);

            // Save the best transformation matrices
            if (overlappings > best_result)
            {
                best_result = overlappings;
                best_rotation = temp_rotation;
                best_translation = temp_translation;
                best_scale = temp_scale;

            }
        }

        ApplyRigidTransformation(best_rotation, best_translation,  best_scale);

    }


    private int CountOverlappings(Matrix<double> temp_rotation, Vector<double> temp_translation, double scale)
    {
        Vector<double> transformedPoint;

        int overlapped_points = 0;
        for (int i = 0; i < P.Length; i++)
        {

            if (isScalable)
                transformedPoint = P[i] * scale * temp_rotation  + temp_translation;
            
            else transformedPoint = P[i] * temp_rotation + temp_translation;

            if (IsMatched(transformedPoint))
                overlapped_points++;            
        }

        return overlapped_points;
    }

    private bool IsMatched(Vector<double> transformedPoint)
    {

        for (int i = 0; i < Q.Length; i++)
        {
            Vector<double> difference = transformedPoint - Q[i];
            double distance = difference.L2Norm();

            if (distance < thresh_hold)
                return true;
        }

        return false;
    }


    private void GetRigidTransformation(Matrix<double> A, Matrix<double> B, out Matrix<double> rotation, out Vector<double> translation, out double scale)
    {
        scale = 1;


        Vector<double> centroid_A = Vector<double>.Build.DenseOfArray(new[] { A.Row(0).Mean(), A.Row(1).Mean(), A.Row(2).Mean() });
        Vector<double> centroid_B = Vector<double>.Build.DenseOfArray(new[] { B.Row(0).Mean(), B.Row(1).Mean(), B.Row(2).Mean() });


        var p = Matrix<double>.Build.DenseOfColumns(new[] { A.Column(0) - centroid_A, A.Column(1) - centroid_A, A.Column(1) - centroid_A });
        var q = Matrix<double>.Build.DenseOfColumns(new[] { B.Column(0) - centroid_B, B.Column(1) - centroid_B, B.Column(1) - centroid_B });


        if (isScalable)
        {
            scale = CalculateScaleFactor(p,q);

            if (scale == 0)
                scale = 1;
            A *= scale;
            centroid_A = Vector<double>.Build.DenseOfArray(new[] { A.Row(0).Mean(), A.Row(1).Mean(), A.Row(2).Mean() });
            p = Matrix<double>.Build.DenseOfColumns(new[] { A.Column(0) - centroid_A, A.Column(1) - centroid_A, A.Column(1) - centroid_A });
        }



        var H = p * q.Transpose();
        var svd = H.Svd(true);

        Matrix<double> U = svd.U;
        Vector<double> S = svd.S;
        Matrix<double> VT = svd.VT;

        Matrix<double> V = VT.Transpose();
        Matrix<double> R = V * U.Transpose();

        if (R.Determinant() < 0)
        {

            VT.Column(2).Multiply(-1);
            V = VT.Transpose();
            R = V * U.Transpose();
        }

        translation = centroid_B - R * centroid_A;
        rotation = R;
    }


    private double CalculateScaleFactor(Matrix<double>  A, Matrix<double> B)
    {
        double sumOfSquaredDistances = 0;

        for (int i = 0; i < 3; i++)
        {
            double distance1 = A.Column(i).L2Norm();
            double distance2 = B.Column(i).L2Norm();

            if (distance1 != 0)
                sumOfSquaredDistances += Math.Pow(distance2 / distance1, 2);
        }

        return Math.Sqrt(sumOfSquaredDistances / 3);
    }

    private void ApplyRigidTransformation(Matrix<double> best_rotation,Vector<double> best_translation, double best_scale)
    {

        alignedSpheres = new GameObject[P.Length];
        Vector<double> alignedP;


        for (int i = 0; i < P.Length; i++)
        {

            if (isScalable)
                alignedP = P[i] * best_rotation * best_scale + best_translation;
            else
                alignedP = P[i] * best_rotation + best_translation;

            alignedSpheres[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            alignedSpheres[i].transform.position = new Vector3((float)alignedP[0], (float)alignedP[1], (float)alignedP[2]);
            alignedSpheres[i].GetComponent<Renderer>().material = alignedPointsMaterial;
            alignedSpheres[i].name = "PQ" + i;


            
            var lineRenderer = alignedSpheres[i].gameObject.AddComponent<LineRenderer>();
            lineRenderer.startWidth = 0.1f;
            lineRenderer.endWidth = 0.1f;
            lineRenderer.SetPosition(0, new Vector3((float)P[i][0], (float)P[i][1], (float)P[i][2]));
            lineRenderer.SetPosition(1, alignedSpheres[i].transform.position);
            lineRenderer.enabled = false;
            alignedSpheres[i].GetComponent<MeshRenderer>().enabled = false;
            //lineRenderer.gameObject.SetActive(false);
        }
        Debug.Log("Rotation: " +best_rotation);
        Debug.Log("Translation: " + best_translation);
        Debug.Log("Scale: " + best_scale);


    }


    public void ShowAlignedPoints()
    {
        for (int i = 0; i < alignedSpheres.Length; i++)
        {
            
                alignedSpheres[i].GetComponent<MeshRenderer>().enabled = !alignedSpheres[i].GetComponent<MeshRenderer>().enabled;
        }

    }

    public void ShowMovementLines()
    {
        for (int i = 0; i < alignedSpheres.Length; i++)
        {
            alignedSpheres[i].GetComponent<LineRenderer>().enabled = !alignedSpheres[i].GetComponent<LineRenderer>().enabled;
        }
    }

}
  