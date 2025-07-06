# Application-4
Engineering Analysis Questions (in your README)
Signal Discipline: How does the binary semaphore synchronize the button task with the system? What happens if the button is pressed multiple times quickly? How might this differ if you’d used a counting semaphore instead?
The binary semaohore sem_emergency_buttons 

Event Flood Handling: When you adjust the potentiometer rapidly above/below threshold, what behavior do you observe with the red LED and console prints? How does the counting semaphore handle multiple signals? What would break if you swapped it for a binary semaphore?

Protecting Shared Output: Describe a moment where removing the mutex caused incorrect console behavior or other issues. Why do mutexes matter for protecting output? What real-world failure could occur if logs were interleaved or shared state was modified concurrently?

Scheduling and Preemption: Did task priorities influence system responsiveness as expected? Give one example where a high-priority task preempted a lower one. What happened to the heartbeat during busy periods?

Timing and Responsiveness: The code provided uses `vTaskDelay` rather than `vTaskDelayUntil`. How did delays impact system responsiveness and behavior? Does the your polling rate affect event detection? Would you consider changing any of the `vTaskDelay` rather than `vTaskDelayUntil` - why or why not? Adjust your code accordingly.

Theme Integration: Connect your implementation to your chosen theme. What does each task/LED/semaphore represent in that real-world system? How might synchronization be life-critical in your domain?

[Bonus] Induced Failure: Can you create a situation where the system starves one or more tasks? (E.g., block the heartbeat for more than 3 seconds, or drop button events.) What caused it? Leave your code commented in the project with an explanation.
