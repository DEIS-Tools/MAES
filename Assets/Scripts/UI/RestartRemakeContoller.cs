using System.Collections;
using System.Collections.Generic;
using Maes;
using UnityEngine;
using UnityEngine.UI;

public class RestartRemakeContoller : MonoBehaviour
{

    public Button RestartCurrentButton;
    public Button RestartAllButton;
    public Button MakeAndRunButton;
    public Button CreateBatchButton;
    public SimulationManager simulationManager;

    private Simulation previousSimulation;

    

    // Start is called before the first frame update
    void Start()
    {
        RestartCurrentButton.onClick.AddListener(() => {
            RestartCurrentScenario();
        });

        RestartAllButton.onClick.AddListener(() => {
            RestartAllScenarios();
        });

        MakeAndRunButton.onClick.AddListener(() => {

        });

        CreateBatchButton.onClick.AddListener(() => {

        });
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void RestartCurrentScenario() {
        Queue<SimulationScenario> newScenariosQueue = new Queue<SimulationScenario>();
        newScenariosQueue.Enqueue(simulationManager._currentScenario);
        simulationManager.RemoveCurrentSimulation();
        if (simulationManager._scenarios.Count != 0) {
            while (simulationManager._scenarios.Count != 0) {
                newScenariosQueue.Enqueue(simulationManager._scenarios.Dequeue());
            }
            simulationManager._scenarios = newScenariosQueue;
        } else {
            simulationManager.CreateSimulation(simulationManager._currentScenario);
        }

        //Basically adds the same simulation to the front of the queue again
        //Second time it get a crash, for some reason

    }
    private void RestartAllScenarios() {
        Queue<SimulationScenario> tempScenariosQueue = new Queue<SimulationScenario>();
        foreach (var scenario in simulationManager._initialScenarios){
            tempScenariosQueue.Enqueue(scenario);
        }
        simulationManager.RemoveCurrentSimulation();

        simulationManager._scenarios = tempScenariosQueue;
    }
    private void MakeAndRunSinglePopup() {

    }
    private void MakeBatchPopup() {

    }
}
