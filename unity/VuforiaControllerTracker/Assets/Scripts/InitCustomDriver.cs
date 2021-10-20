using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;

public class InitCustomDriver : MonoBehaviour
{
    private void Awake()
    {
        VuforiaUnity.SetDriverLibrary("DriverTemplate.dll");
        VuforiaRuntime.Instance.InitVuforia();
    }
}
