"""Bounded Nav2 configuration, then hand activation/bonds to the vendor manager.

Humble's manager can wait indefinitely for a lost change_state response. Read
back lifecycle state before considering another transition. No velocity/goals.
"""
import time

NODES=('controller_server','smoother_server','planner_server','behavior_server','bt_navigator','velocity_smoother')

def configure(name,read,change,*,clock=time.monotonic,pause=time.sleep,timeout=15.):
    deadline=clock()+timeout;requested=False
    while clock()<deadline:
        state=read()
        if state==2:return  # INACTIVE: configured, including a lost reply.
        if state==3:raise RuntimeError('NAV2_UNEXPECTED_ACTIVE '+name)
        if state==1 and not requested:
            requested=True
            try:accepted=change()
            except TimeoutError:accepted=None
            if accepted is False:raise RuntimeError('NAV2_CONFIGURE_REJECTED '+name)
        elif state not in (None,1,10):raise RuntimeError('NAV2_INVALID_STATE '+name+' '+str(state))
        pause(.1)
    raise RuntimeError('NAV2_CONFIGURE_TIMEOUT '+name)

def main():
    import rclpy
    from lifecycle_msgs.srv import GetState,ChangeState
    from nav2_msgs.srv import ManageLifecycleNodes
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('mediassist_nav2_startup')
    def call(client,request,timeout):
        if not client.wait_for_service(timeout_sec=timeout):raise TimeoutError('service discovery')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():
            future.cancel();raise TimeoutError('service response')
        result=future.result()
        if result is None:raise TimeoutError('empty service response')
        return result
    try:
        for name in NODES:
            state=node.create_client(GetState,'/'+name+'/get_state')
            transition=node.create_client(ChangeState,'/'+name+'/change_state')
            def read():
                try:return call(state,GetState.Request(),1.).current_state.id
                except TimeoutError:return None
            def change():
                req=ChangeState.Request();req.transition.id=1  # CONFIGURE only.
                return call(transition,req,2.).success
            node.get_logger().info('NAV2_CONFIGURE_BOUNDED '+name)
            configure(name,read,change)
            node.get_logger().info('NAV2_CONFIGURED_STATE_CONFIRMED '+name)
        manager=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
        active=node.create_client(Trigger,'/lifecycle_manager_navigation/is_active')
        if not manager.wait_for_service(timeout_sec=10.):raise RuntimeError('NAV2_MANAGER_UNAVAILABLE')
        req=ManageLifecycleNodes.Request();req.command=ManageLifecycleNodes.Request.RESUME
        future=manager.call_async(req);deadline=time.monotonic()+180.
        node.get_logger().info('NAV2_ACTIVATING_WITH_VENDOR_BONDS; map TF/initial pose may still be needed')
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if future.done():
                if future.result() is None or not future.result().success:raise RuntimeError('NAV2_ACTIVATION_REJECTED')
                node.get_logger().info('NAV2_ACTIVE_WITH_VENDOR_BONDS');return
            # Also covers a lost final RESUME response, but only when the stock
            # manager itself confirms activation and all its bonds are established.
            try:
                if call(active,Trigger.Request(),1.).success:
                    node.get_logger().info('NAV2_ACTIVE_WITH_VENDOR_BONDS');return
            except TimeoutError:pass
            time.sleep(.5)
        raise RuntimeError('NAV2_ACTIVATION_TIMEOUT')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
