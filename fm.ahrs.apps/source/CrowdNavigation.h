#pragma once
//
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "base/BaseApp.h"
#include "concurrentqueue/concurrentqueue.h"
#include "thread/thread_callback.h"
#include <rapidcsv.h>
//
namespace Urho3D
{
    class Drawable;
    class Node;
    class Scene;
}
//
class CrowdNavigation : public BaseApp
{
    URHO3D_OBJECT( CrowdNavigation, BaseApp );
public:
    /// Construct.
    explicit CrowdNavigation( Context* context );

    /// Setup after engine initialization and before running the main loop.
    void Start() override;
protected:
    /// Return XML patch instructions for screen joystick layout for a specific sample app, if any.
    String GetScreenJoystickPatchString() const override
    {
        return "<patch>"
               "    <add sel=\"/element\">"
               "        <element type=\"Button\">"
               "            <attribute name=\"Name\" value=\"Button3\" />"
               "            <attribute name=\"Position\" value=\"-120 -120\" />"
               "            <attribute name=\"Size\" value=\"96 96\" />"
               "            <attribute name=\"Horiz Alignment\" value=\"Right\" />"
               "            <attribute name=\"Vert Alignment\" value=\"Bottom\" />"
               "            <attribute name=\"Texture\" value=\"Texture2D;Textures/TouchInput.png\" />"
               "            <attribute name=\"Image Rect\" value=\"96 0 192 96\" />"
               "            <attribute name=\"Hover Image Offset\" value=\"0 0\" />"
               "            <attribute name=\"Pressed Image Offset\" value=\"0 0\" />"
               "            <element type=\"Text\">"
               "                <attribute name=\"Name\" value=\"Label\" />"
               "                <attribute name=\"Horiz Alignment\" value=\"Center\" />"
               "                <attribute name=\"Vert Alignment\" value=\"Center\" />"
               "                <attribute name=\"Color\" value=\"0 0 0 1\" />"
               "                <attribute name=\"Text\" value=\"Spawn\" />"
               "            </element>"
               "            <element type=\"Text\">"
               "                <attribute name=\"Name\" value=\"KeyBinding\" />"
               "                <attribute name=\"Text\" value=\"LSHIFT\" />"
               "            </element>"
               "            <element type=\"Text\">"
               "                <attribute name=\"Name\" value=\"MouseButtonBinding\" />"
               "                <attribute name=\"Text\" value=\"LEFT\" />"
               "            </element>"
               "        </element>"
               "        <element type=\"Button\">"
               "            <attribute name=\"Name\" value=\"Button4\" />"
               "            <attribute name=\"Position\" value=\"-120 -12\" />"
               "            <attribute name=\"Size\" value=\"96 96\" />"
               "            <attribute name=\"Horiz Alignment\" value=\"Right\" />"
               "            <attribute name=\"Vert Alignment\" value=\"Bottom\" />"
               "            <attribute name=\"Texture\" value=\"Texture2D;Textures/TouchInput.png\" />"
               "            <attribute name=\"Image Rect\" value=\"96 0 192 96\" />"
               "            <attribute name=\"Hover Image Offset\" value=\"0 0\" />"
               "            <attribute name=\"Pressed Image Offset\" value=\"0 0\" />"
               "            <element type=\"Text\">"
               "                <attribute name=\"Name\" value=\"Label\" />"
               "                <attribute name=\"Horiz Alignment\" value=\"Center\" />"
               "                <attribute name=\"Vert Alignment\" value=\"Center\" />"
               "                <attribute name=\"Color\" value=\"0 0 0 1\" />"
               "                <attribute name=\"Text\" value=\"Obstacles\" />"
               "            </element>"
               "            <element type=\"Text\">"
               "                <attribute name=\"Name\" value=\"MouseButtonBinding\" />"
               "                <attribute name=\"Text\" value=\"MIDDLE\" />"
               "            </element>"
               "        </element>"
               "    </add>"
               "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/attribute[@name='Is Visible']\" />"
               "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Set</replace>"
               "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]\">"
               "        <element type=\"Text\">"
               "            <attribute name=\"Name\" value=\"MouseButtonBinding\" />"
               "            <attribute name=\"Text\" value=\"LEFT\" />"
               "        </element>"
               "    </add>"
               "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/attribute[@name='Is Visible']\" />"
               "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Debug</replace>"
               "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]\">"
               "        <element type=\"Text\">"
               "            <attribute name=\"Name\" value=\"KeyBinding\" />"
               "            <attribute name=\"Text\" value=\"SPACE\" />"
               "        </element>"
               "    </add>"
               "</patch>";
    }
private:
    /// Construct the scene content.
    void CreateScene();
    /// Construct user interface elements.
    void CreateUI();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to application-wide logic update and post-render update events.
    void SubscribeToEvents();
    /// Read input and moves the camera.
    void MoveCamera( float timeStep );
    /// Set crowd agents target or spawn another jack.
    void SetPathPoint( int spawning );
    /// Add new obstacle or remove existing obstacle/agent.
    void AddOrRemoveObject();
    /// Create a "Jack" object at position.
    void SpawnJack( const Vector3& pos, Node* jackGroup );
    /// Create a mushroom object at position.
    void CreateMushroom( const Vector3& pos );
    /// Create an off-mesh connection for each box to make it climbable.
    void CreateBoxOffMeshConnections( DynamicNavigationMesh* navMesh, Node* boxGroup );
    /// Create some movable barrels as crowd agents.
    void CreateMovingBarrels( DynamicNavigationMesh* navMesh );
    /// Utility function to raycast to the cursor position. Return true if hit.
    bool Raycast( float maxDistance, Vector3& hitPos, Drawable*& hitDrawable );
    /// Toggle navigation mesh streaming.
    void ToggleStreaming( bool enabled );
    /// Update navigation mesh streaming.
    void UpdateStreaming();
    /// Save navigation data for streaming.
    void SaveNavigationData();
    /// Handle the logic update event.
    void HandleUpdate( StringHash eventType, VariantMap& eventData );
    /// Handle the post-render update event.
    void HandlePostRenderUpdate( StringHash eventType, VariantMap& eventData );
    /// Handle problems with crowd agent placement.
    void HandleCrowdAgentFailure( StringHash eventType, VariantMap& eventData );
    /// Handle crowd agent reposition.
    void HandleCrowdAgentReposition( StringHash eventType, VariantMap& eventData );
    /// Handle crowd agent formation.
    void HandleCrowdAgentFormation( StringHash eventType, VariantMap& eventData );
    void HandleQuit( StringHash eventType, VariantMap& eventData );
    //
    //
    void setNewPos( float x, float y, float z );
    //
    void init_sensor();
    //
    void read_sensor_start();
    //
    void read_sensor_end();
    //
    void init_out_csv( const std::string& filename );
    //
    void update_out_csv( const SENSOR_DB& sensor_data );
    //
    void close_out_csv();
    //
    void read_in_csv( const std::string& filename );
public:
    /// Flag for using navigation mesh streaming.
    bool useStreaming_{};
    /// Streaming distance.
    int streamingDistance_{ 2 };
    /// Tile data.
    HashMap< IntVector2, Vector< byte > > tileData_;
    /// Added tiles.
    HashSet< IntVector2 > addedTiles_;
    /// Flag for drawing debug geometry.
    bool drawDebug_{};
    /// Instruction text UI-element.
    Text* instructionText_{};
    Text* infoText_{};
    //
    Vector3   hitPos;
    Drawable* hitDrawable;
    Node*     axes_node_;
public:
    bool              mul_views_{ false };
    const std::string i2cDevice         = "/dev/i2c-1";
    uint8_t           deviceAddress_mmc = 0x30;
    uint8_t           deviceAddress_imu = 0x69;
    float             elapsedTime_;
    //
    MMC56x3          sensor_mmc_;
    ICM42670         sensor_imu_;
    AhrsCalculation* ahrs_calculation_;
    //
    moodycamel::ConcurrentQueue< SENSOR_DB > sensor_data_queue_;
    //
    // 创建一个 Document 对象，用于写入 CSV 文件
    rapidcsv::Document csv_doc_;
    std::string        csv_filename_;
    int                csv_index_{ 0 };
public:
    int app_mode_{ 0 };  // 0: Measurement mode, 1:Simulation Mode
};
