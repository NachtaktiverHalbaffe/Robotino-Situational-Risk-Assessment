import requests
import rospy

from .constants import Metrics, LoggingLevel


class EvalManagerClient:
    """
    This is basically a wrapper for sending the various requests to Evaluationmanager via the REST API. For each logging usecase\
    a own function is given which creates the corresponding payloads and sending them to the right endpoints. Additionally\
    the text logging messages are standardized here so they are potentially automatically processable by the evaluationmanager
    """

    def __init__(
        self, host: str = "129.69.102.129", port: int = 5000, timeout: float = 2
    ):
        self.baseUri = f"http://{host}:{port}"
        self.payload = {}
        self.timeout = timeout

    def evalLogCollision(self, stepSize: int = 1):
        """
        Sends a request to the evaluationmanager to add an collision

        Args:
            stepSize (int,optional): How much collisions should be added
        """
        self.payload = {
            "metric": Metrics.COLLISIONS.value,
            "loggerSource": "prototype",
            "stepSize": stepSize,
        }

        uri = f"{self.baseUri}/increaselogger"
        self.__sendRequest(url=uri)

    def evalLogStop(self, stepSize: int = 1):
        """
        Sends a request to the evaluationmanager to add an stop of the robot

        Args:
            stepSize (int,optional): How much stops should be added
        """
        self.payload = {
            "metric": Metrics.PRODUCTION_STOPS.value,
            "loggerSource": "prototype",
            "stepSize": stepSize,
        }

        uri = f"{self.baseUri}/increaselogger"
        self.__sendRequest(url=uri)

    def evalLogSuccessfulTask(self, stepSize: int = 1):
        """
        Sends a request to the evaluationmanager to add an successful task

        Args:
            stepSize (int,optional): How much stops should be added
        """
        self.payload = {
            "metric": Metrics.SUCCESSFUL_TASKS.value,
            "loggerSource": "prototype",
            "stepSize": stepSize,
        }

        uri = f"{self.baseUri}/increaselogger"
        self.__sendRequest(url=uri)

    def evalLogStartedTask(self, stepSize: int = 1):
        """
        Sends a request to the evaluationmanager to add an started task

        Args:
            stepSize (int,optional): How much stops should be added
        """
        self.payload = {
            "metric": Metrics.STARTED_TASKS.value,
            "loggerSource": "prototype",
            "stepSize": stepSize,
        }

        uri = f"{self.baseUri}/increaselogger"
        self.__sendRequest(url=uri)

    def evalLogRisk(self, risk: float):
        """
        Sends a request to the evaluationmanager to add an risk

        Args:
            risk (float): The risk value itself
        """
        self.payload = {
            "metric": Metrics.RISK.value,
            "loggerSource": "prototype",
            "value": float(risk),
        }

        uri = f"{self.baseUri}/valuelogger"
        self.__sendRequest(url=uri)

    def evalLogCollisionProb(self, prob: float):
        """
        Sends a request to the evaluationmanager to add an risk

        Args:
            risk (float): The risk value itself
        """
        self.payload = {
            "metric": Metrics.COLLISION_PROBABILITY.value,
            "loggerSource": "prototype",
            "value": prob,
        }

        uri = f"{self.baseUri}/valuelogger"
        self.__sendRequest(url=uri)

    def evalLogNav(
        self,
        x: int = None,
        y: int = None,
        id: int = None,
        level: str = LoggingLevel.INFO.value,
        success: bool = True,
    ):
        """
        Sends a request to the evaluationmanager to add an textlog if Robotino did complete navigation successfully or not

        Args:
            x (int, optional): x-coordinate uif target was an coordinate
            y (int, optional): y-coordinate if target was an coordinate
            id (int, optional): Id of targeted workstation if target was an workstation
            level (str, optional): The logging level which should be used. Defaults to INFO
            success (bool, optional): If it is a success (True) or error response (False)
        """

        if x != None and y != None:
            targetIdentifier = f"(x:{x},y:{y})"
        elif id != None:
            targetIdentifier = f"(id:{id})"
        else:
            rospy.logwarn(
                f"[Evalmanager Client] Couldn't send navLog: Neither coordinate nor id given"
            )
            return
        if success:
            msg = f"Success: Robotino reached target {targetIdentifier}"
        else:
            msg = f"Error: Robotino didn't reached target {targetIdentifier}"
        self.payload = {"msg": msg, "level": level}

        uri = f"{self.baseUri}/textlogger"
        self.__sendRequest(url=uri)

    def evalLogCriticalSector(
        self, x: int, y: int, localRisk: float, level: str = LoggingLevel.INFO.value
    ):
        """
        Sends a request to the evaluationmanager that Robotino has entered a sector with an potential collision risk

        Args:
          x (int): x-coordinate of the target node
          y (int): y-coordinate of the target node
          localRisk (float): The calculated risk for the sector
          level (str, optional): The logging level which should be used. Defaults to INFO
        """
        msg = f"Robotino is entering critical sector. Params: x:{x} y:{y} risk:{localRisk}"
        self.payload = {"msg": msg, "level": level}

        uri = f"{self.baseUri}/textlogger"
        self.__sendRequest(url=uri)

    def evalLogUncriticalSector(
        self, x: int, y: int, level: str = LoggingLevel.INFO.value
    ):
        """
        Sends a request to the evaluationmanager that Robotino has entered a sector without an potential collision risk

        Args:
          x (int): x-coordinate of the target node
          y (int): y-coordinate of the target node
          localRisk (float): The calculated risk for the sector
          level (str, optional): The logging level which should be used. Defaults to INFO
        """
        msg = f"Robotino is entering uncritical sector. Params: x:{x} y:{y} risk:0"
        self.payload = {"msg": msg, "level": level}

        uri = f"{self.baseUri}/textlogger"
        self.__sendRequest(url=uri)

    def evalLogLIDAR(self, enabled: bool, level: str = LoggingLevel.INFO.value):
        """
        Sends a request to the evaluationmanager if LIDAR was enabled or disabled

        Args:
            enabled (bool): If LIDAR was enabled (True) or disabled (False)
            level (str, optional): The logging level which should be used. Defaults to INFO
        """
        if enabled:
            msg = "LIDAR status changed: Running"
        else:
            msg = "LIDAR status changed: Down"
        self.payload = {"msg": msg, "level": level}

        uri = f"{self.baseUri}/textlogger"
        self.__sendRequest(url=uri)

    def __sendRequest(self, url: str):
        """
        Wrapper for a Post request with a timeout
        """
        try:
            response = requests.post(url=url, json=self.payload, timeout=self.timeout)
            # print(response.content)
        except requests.exceptions.Timeout:
            return
        except Exception as e:
            rospy.logwarn(f"[Evalmanager Client] Couldn't send request to {url}: {e}")
