import airsim

def main():
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("API Control enabled: %s" % client.isApiControlEnabled())

    response = client.getSonarData("Sonar", "RovSimple")
    print("Sonar data: %s" % response.image)

if __name__ == "__main__":
    main()